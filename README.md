# spatial turbulence algorithm on extracted amplitude data from 12 subcarriers over 2.4GHz

A lightweight algorithm that measures spatial turbulence using amplitude fluctuations across 12 subcarriers on the 2.4 GHz band.

When physical movement occurs in an RF environment, channel amplitudes shift unevenly. This tool processes those raw amplitude vectors over time to compute a single "turbulence" score in a ultra-low-cost way.

### How it Works

* **Subcarrier Vector Input:** Reads 12 subcarrier amplitude channels per frame.
* **Noise Baseline Filtering:** Strips out static attenuation to focus purely on dynamic variance.
* **Spatial Dispersion Score:** Calculates cross-subcarrier variance across sliding time windows to output a turbulence index.

### Input Data Format

* **Subcarriers:** 12 channels (equidistant or custom indices)
* **Frequency:** 2.4 GHz band
* **Values:** Raw linear proprietary format
