# RM3100 Serial Test for nRF52840

Standalone ItsyBitsy nRF52840 sketch for checking the RM3100 after the PCB change.

What it does:

- initializes the existing RM3100 driver code
- reads the magnetometer in single-shot mode
- prints raw X/Y/Z counts and converted microtesla values over serial

How to use:

1. Open this folder as a PlatformIO project.
2. Build and upload to the nRF module.
3. Open the serial monitor at 115200 baud.
4. Verify that the readings change when you rotate the board.

Expected output looks like:

```text
RM3100 serial test starting on nRF52840
Measurement time (ms): 57.600
raw x=... y=... z=... | uT x=... y=... z=...
```
