#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_wifi_types.h>
#include <lwip/sockets.h>
#include <lwip/netdb.h>
#include <algorithm>
#include <cmath>

//config
#define CALIB_PACKETS       300     // Calibration window (~3 seconds @ 100Hz)[cite: 2]
#define WINDOW_SIZE         100     // Moving variance window size[cite: 2]
#define HAMPEL_WINDOW       7       // Hampel filter window size[cite: 2]
#define HAMPEL_THRESHOLD    5.0f    // Hampel filter MAD multiplier threshold[cite: 2]
#define DEFAULT_THRESHOLD   0.0018f // Tuned default CV threshold for skipped calibration[cite: 2]

// 12 subC
const uint8_t TARGET_SUBCARRIERS[12] = {12, 14, 16, 18, 20, 24, 28, 36, 40, 44, 48, 52};

enum PipelineState {
  STATE_CALIBRATING,
  STATE_RUNNING
};

struct CsiPacket {
  int8_t  rssi;
  int8_t  noise_floor;
  int8_t  raw_buf[128];
  uint16_t len;
};

// icmp for ping
struct icmp_hdr {
  uint8_t  type;
  uint8_t  code;
  uint16_t checksum;
  uint16_t id;
  uint16_t seq;
};

PipelineState currentState = STATE_CALIBRATING;
QueueHandle_t csiQueue;

uint32_t packetCount = 0;
uint32_t totalProcessedPackets = 0;
uint32_t lastPrintTime = 0;

// buffer check values
float turbulenceBuffer[WINDOW_SIZE];
int   turbIdx = 0;
int   turbCount = 0;

float hampelBuffer[HAMPEL_WINDOW];
int   hampelIdx = 0;
int   hampelCount = 0;

float baselineMvAccumulator = 0.0f;
uint32_t baselineMvSamples = 0;
float adaptiveThreshold = DEFAULT_THRESHOLD;
bool  motionState = false;

// filter remove median (page4)
float applyHampelFilter(float val) {
  hampelBuffer[hampelIdx] = val;
  hampelIdx = (hampelIdx + 1) % HAMPEL_WINDOW;
  if (hampelCount < HAMPEL_WINDOW) hampelCount++;
  if (hampelCount < 3) return val;

  float sorted[HAMPEL_WINDOW];
  for (int i = 0; i < hampelCount; i++) sorted[i] = hampelBuffer[i];
  std::sort(sorted, sorted + hampelCount);
  float median = sorted[hampelCount / 2];

  float dev[HAMPEL_WINDOW];
  for (int i = 0; i < hampelCount; i++) dev[i] = std::abs(hampelBuffer[i] - median);
  std::sort(dev, dev + hampelCount);
  float mad = dev[hampelCount / 2];

  if (mad > 1e-6f) {
    float threshold_scaled = HAMPEL_THRESHOLD * 1.4826f;
    if (std::abs(val - median) / mad > threshold_scaled) {
      return median; // idk?
    }
  }
  return val;
}

// 2pass go
float computeMovingVariance(float newVal) {
  turbulenceBuffer[turbIdx] = newVal;
  turbIdx = (turbIdx + 1) % WINDOW_SIZE;
  if (turbCount < WINDOW_SIZE) turbCount++;

  if (turbCount < 2) return 0.0f;

  int n = turbCount;
  float mean = 0.0f;
  for (int i = 0; i < n; i++) mean += turbulenceBuffer[i];
  mean /= n;

  float varSum = 0.0f;
  for (int i = 0; i < n; i++) {
    float diff = turbulenceBuffer[i] - mean;
    varSum += diff * diff;
  }
  return varSum / n;
}

// icmp ping checksum for true cond
uint16_t calculateChecksum(uint16_t *buf, int len) {
  uint32_t sum = 0;
  while (len > 1) {
    sum += *buf++;
    len -= 2;
  }
  if (len == 1) {
    sum += *(uint8_t*)buf;
  }
  sum = (sum >> 16) + (sum & 0xFFFF);
  sum += (sum >> 16);
  return (uint16_t)(~sum);
}

//harware part
void IRAM_ATTR wifiCsiCallback(void* ctx, wifi_csi_info_t* data) {
  if (!data || !data->buf) return;

  CsiPacket pkt;
  pkt.rssi = data->rx_ctrl.rssi;
  pkt.noise_floor = data->rx_ctrl.noise_floor;
  
  uint16_t copyLen = (data->len < 128) ? data->len : 128;
  memcpy(pkt.raw_buf, data->buf, copyLen);
  pkt.len = copyLen;

  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xQueueSendFromISR(csiQueue, &pkt, &xHigherPriorityTaskWoken);
  if (xHigherPriorityTaskWoken) {
    portYIELD_FROM_ISR();
  }
}

//ddos with ping
void trafficGeneratorTask(void* pvParameters) {
  vTaskDelay(pdMS_TO_TICKS(200));

  IPAddress gw = WiFi.gatewayIP();
  int sock = socket(AF_INET, SOCK_RAW, IPPROTO_ICMP);
  
  if (sock < 0) {
    Serial.println("[TRAFFIC] RAW socket failed. Falling back to UDP probe...");
    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
  }

  sockaddr_in destAddr;
  destAddr.sin_addr.s_addr = inet_addr(gw.toString().c_str());
  destAddr.sin_family = AF_INET;

  uint16_t seq = 0;
  char packetBuf[64];

  Serial.printf("[TRAFFIC] Injecting ICMP Ping requests into Gateway (%s)...\n", gw.toString().c_str());

  while (true) {
    if (WiFi.status() == WL_CONNECTED) {
      memset(packetBuf, 0, sizeof(packetBuf));
      icmp_hdr *icmp = (icmp_hdr*)packetBuf;
      icmp->type = 8;
      icmp->code = 0;
      icmp->id = htons(0x1234);
      icmp->seq = htons(seq++);
      icmp->checksum = 0;
      icmp->checksum = calculateChecksum((uint16_t*)packetBuf, sizeof(packetBuf));

      sendto(sock, packetBuf, sizeof(packetBuf), 0, (struct sockaddr*)&destAddr, sizeof(destAddr));
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

//process res
void dspProcessingTask(void* pvParameters) {
  CsiPacket pkt;

  while (true) {
    if (xQueueReceive(csiQueue, &pkt, portMAX_DELAY)) {
      packetCount++;
      totalProcessedPackets++;

      // turbulance filte
      float amplitudes[12];
      float ampSum = 0.0f;

      for (int i = 0; i < 12; i++) {
        uint8_t scIdx = TARGET_SUBCARRIERS[i];
        int offset = scIdx * 2;
        if (offset + 1 < pkt.len) {
          int8_t q = pkt.raw_buf[offset];
          int8_t i_val = pkt.raw_buf[offset + 1];
          amplitudes[i] = std::sqrt((float)(i_val * i_val + q * q)); // page8
        } else {
          amplitudes[i] = 0.0f;
        }
        ampSum += amplitudes[i];
      }

      float meanAmp = ampSum / 12.0f;
      if (meanAmp < 1e-4f) continue;

      float sqDiffSum = 0.0f;
      for (int i = 0; i < 12; i++) {
        float diff = amplitudes[i] - meanAmp;
        sqDiffSum += diff * diff;
      }
      float stdDev = std::sqrt(sqDiffSum / 12.0f);

      // CV = std / mean page 10
      float cvTurbulence = stdDev / meanAmp;

      // Hampel Filter page 11
      float filteredTurbulence = applyHampelFilter(cvTurbulence);

      // Moving Variance page 11
      float mv = computeMovingVariance(filteredTurbulence);

      // calib check yes
      if (currentState == STATE_CALIBRATING) {
        if (packetCount > 10) {
          baselineMvAccumulator += mv;
          baselineMvSamples++;
        }

        if (packetCount % 50 == 0) {
          Serial.printf("[CALIBRATING] Received %d / %d packets...\n", packetCount, CALIB_PACKETS);
        }

        if (packetCount >= CALIB_PACKETS) {
          float avgBaselineMv = (baselineMvSamples > 0) ? (baselineMvAccumulator / baselineMvSamples) : 0.0005f;
          
          // x1.6 work good with max 10m, x1 max 25m, x< for closer to ap
          adaptiveThreshold = avgBaselineMv * 1.6f;
          if (adaptiveThreshold < 0.0005f) adaptiveThreshold = 0.0005f;

          Serial.println("\n===================================================================");
          Serial.printf("  CALIBRATION COMPLETE\n");
          Serial.printf("  Baseline Variance : %.6f\n", avgBaselineMv);
          Serial.printf("  Adaptive Threshold: %.6f\n", adaptiveThreshold);
          Serial.println("===================================================================\n");

          currentState = STATE_RUNNING;
        }
        continue;
      }

      //estimate
      if (currentState == STATE_RUNNING) {
        motionState = (mv > adaptiveThreshold);

        uint32_t now = millis();
        if (now - lastPrintTime >= 200) {
          lastPrintTime = now;
          float sensRatio = (adaptiveThreshold > 0) ? (mv / adaptiveThreshold * 100.0f) : 0.0f;
          
          Serial.printf("[LIVE] Pkts: %5u | MV: %.6f | Thresh: %.6f | Sens: %3.0f%% | State: %s\n",
                        totalProcessedPackets, mv, adaptiveThreshold, sensRatio, 
                        motionState ? ">>> MOTION DETECTED <<<" : "IDLE");
        }
      }
    }
  }
}

// ask user
void scanAndSelectAP(String &ssid, String &password, int &channel) {
  while (true) {
    Serial.println("\n[WIFI SCANNER] Scanning for Wi-Fi Access Points...");
    int n = WiFi.scanNetworks();

    if (n == 0) {
      Serial.println("[WIFI SCANNER] No networks found. Rescanning in 5 seconds...");
    } else {
      Serial.printf("[WIFI SCANNER] Found %d networks:\n", n);
      Serial.println("-----------------------------------------------------------------------------------");
      for (int i = 0; i < n; ++i) {
        Serial.printf("  [%2d] SSID: %-32s | RSSI: %3d dBm | Ch: %2d | Auth: %s\n",
                      i, 
                      WiFi.SSID(i).c_str(), 
                      WiFi.RSSI(i), 
                      WiFi.channel(i),
                      (WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? "Open" : "Secured");
      }
      Serial.println("-----------------------------------------------------------------------------------");
      Serial.println(">>> Type AP index number [0-N] and press ENTER to select.");
      Serial.println(">>> Rescanning automatically in 5 seconds if no selection is made...\n");

      unsigned long startTime = millis();
      int selectedIndex = -1;

      while (millis() - startTime < 5000) {
        if (Serial.available() > 0) {
          String input = Serial.readStringUntil('\n');
          input.trim();
          if (input.length() > 0) {
            int idx = input.toInt();
            if (idx >= 0 && idx < n) {
              selectedIndex = idx;
              break;
            } else {
              Serial.println("[ERROR] Invalid selection index. Try again.");
            }
          }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
      }

      if (selectedIndex != -1) {
        ssid = WiFi.SSID(selectedIndex);
        channel = WiFi.channel(selectedIndex);
        bool isSecured = (WiFi.encryptionType(selectedIndex) != WIFI_AUTH_OPEN);

        Serial.printf("\n[SELECTED AP] SSID: %s (Channel %d)\n", ssid.c_str(), channel);

        if (isSecured) {
          Serial.print(">>> Enter Password for ");
          Serial.print(ssid);
          Serial.println(" in Serial Monitor and press ENTER:");

          while (Serial.available() == 0) {
            vTaskDelay(pdMS_TO_TICKS(100));
          }
          password = Serial.readStringUntil('\n');
          password.trim();
        } else {
          password = "";
        }
        
        WiFi.scanDelete();
        return;
      }
    }
    WiFi.scanDelete();
  }
}

// ask chatgpt for nice text output
bool askCalibrationPrompt() {
  Serial.println("\n-------------------------------------------------------------------");
  Serial.println(">>> Run baseline calibration? [Y/n] (Auto-Yes in 5s)");
  Serial.println("    [Y] = Calibrate baseline threshold (~3 sec)");
  Serial.println("    [n] = Skip calibration and show live motion numbers immediately");
  Serial.println("-------------------------------------------------------------------");

  unsigned long startTime = millis();
  while (millis() - startTime < 5000) {
    if (Serial.available() > 0) {
      String input = Serial.readStringUntil('\n');
      input.trim();
      if (input.equalsIgnoreCase("n")) {
        Serial.println("[CALIBRATION] Skipped by user. Using default threshold.");
        return false;
      } else if (input.equalsIgnoreCase("y") || input.length() == 0) {
        Serial.println("[CALIBRATION] User confirmed. Starting calibration...");
        return true;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
  Serial.println("[CALIBRATION] Timeout. Proceeding with baseline calibration...");
  return true;
}

//void setup
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n===================================================================");
  Serial.println("  ESP32-S3 High-Speed Wi-Fi CSI Motion Detector (ESPectre Engine)");
  Serial.println("===================================================================");

  
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  
  String selectedSSID = "";
  String selectedPass = "";
  int targetChannel = 0;

  scanAndSelectAP(selectedSSID, selectedPass, targetChannel);

  
  Serial.printf("\n[WIFI] Connecting to '%s'...", selectedSSID.c_str());
  WiFi.begin(selectedSSID.c_str(), selectedPass.c_str());

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.printf("\n[WIFI] Connected! Gateway IP: %s | Channel: %d\n", 
                WiFi.gatewayIP().toString().c_str(), WiFi.channel());

  // ASK RETURN FUNC FOR CALIB
  bool runCalib = askCalibrationPrompt();
  if (runCalib) {
    currentState = STATE_CALIBRATING;
  } else {
    currentState = STATE_RUNNING;
    adaptiveThreshold = DEFAULT_THRESHOLD;
  }

  // BUGGY QUEUE
  csiQueue = xQueueCreate(100, sizeof(CsiPacket));

  wifi_csi_config_t csi_config = {
    .lltf_en = true,
    .htltf_en = true,
    .stbc_htltf2_en = true,
    .ltf_merge_en = true,
    .channel_filter_en = true,
    .manu_scale = false,
    .shift = false
  };

  ESP_ERROR_CHECK(esp_wifi_set_csi_config(&csi_config));
  ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(wifiCsiCallback, NULL));
  ESP_ERROR_CHECK(esp_wifi_set_csi(true));

  Serial.println("[CSI] Hardware receiver enabled.");

  
  xTaskCreatePinnedToCore(trafficGeneratorTask, "TrafficTask", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(dspProcessingTask, "DSPTask", 8192, NULL, 2, NULL, 1);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
