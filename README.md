ESP32 ADXL345 Vibration Sensor (WITS Output)
Simple vibration monitoring system using ESP32 and ADXL345 accelerometer that streams vibration data in WITS format over USB serial.

Designed for low-frequency vibration analysis (2–70 Hz) and integration with systems such as Xview or other WITS parsers.

Hardware
Adafruit QT Py ESP32 Pico

ADXL345 Accelerometer (STEMMA QT / Qwiic)

STEMMA QT cable

Connection is done via STEMMA QT I²C.

ESP32 uses Wire1 (STEMMA QT):

SDA = GPIO 22

SCL = GPIO 19

Sensor I²C address:

0x53

Features
3-axis acceleration measurement

RMS vibration calculation

RMS AC (vibration only, gravity removed)

Dominant frequency detection using FFT

WITS formatted output

Stable frequency detection for 10–20 Hz vibration

Noise protection using RMS threshold

Measurement Parameters
Sampling rate:

200 Hz

RMS window:

1 second

FFT window:

512 samples (~2.56 s)

Frequency detection range:

2–70 Hz

Frequency calculation condition:

Dominant frequency is calculated only if RMS AC ≥ 8 mg

This prevents false frequency detection caused by sensor noise.

WITS Output Format
Data is streamed every 1 second.

Example output:

&&
27118
2712768
2713558
2714950
27158
271624
!!

Meaning of records:

2711 = X axis acceleration (mg)

2712 = Y axis acceleration (mg)

2713 = Z axis acceleration (mg)

2714 = RMS total acceleration

2715 = RMS AC (vibration amplitude)

2716 = Dominant frequency (Hz)

Packet markers:

&& = start of packet

!! = end of packet

RMS Calculation
Total RMS:
RMS = sqrt((x² + y² + z²) / N)

RMS AC removes gravity using mean subtraction:

xAC = x - mean(x)

yAC = y - mean(y)

zAC = z - mean(z)

RMS AC:
RMS_AC = sqrt((xAC² + yAC² + zAC²) / N)

RMS AC represents true vibration amplitude.

Dominant Frequency Detection
Processing steps:

Select axis with highest RMS AC

Remove DC component

Apply Hamming window

Perform FFT calculation

Find peak between 2–70 Hz

Frequency is reported only if:

RMS AC ≥ 8 mg

Otherwise:

2716 = 0

Serial Settings
Baud rate: 115200

Data bits: 8

Parity: none

Stop bits: 1

Output is compatible with systems that parse WITS formatted streams.

Libraries
Required Arduino libraries:

arduinoFFT

Wire

No external ADXL345 library is required because the sensor is read directly via I²C registers.

Typical Use Cases
Camera vibration monitoring

LTS vibration testing

Mechanical vibration diagnostics

Rig instrumentation

Sensor prototyping
