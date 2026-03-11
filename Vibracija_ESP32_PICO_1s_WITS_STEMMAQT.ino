#include <Wire.h>
#include <arduinoFFT.h>
#include <math.h>

#define STEMMA_SDA 22
#define STEMMA_SCL 19
#define ADXL_ADDR  0x53

// ADXL345 registers
#define REG_DEVID       0x00
#define REG_BW_RATE     0x2C
#define REG_POWER_CTL   0x2D
#define REG_DATA_FORMAT 0x31
#define REG_DATAX0      0x32

// Sampling
const uint16_t SAMPLE_RATE_HZ = 200;
const uint32_t SAMPLE_PERIOD_US = 1000000UL / SAMPLE_RATE_HZ;

// 1 s RMS window
const uint16_t RMS_WINDOW = 200;

// FFT window = 2.56 s
const uint16_t FFT_WINDOW = 512;

const float FREQ_MIN = 2.0f;
const float FREQ_MAX = 70.0f;
const float RMS_AC_THRESHOLD = 8.0f;   // mg

// 1 s buffers
float xRmsBuf[RMS_WINDOW];
float yRmsBuf[RMS_WINDOW];
float zRmsBuf[RMS_WINDOW];
uint16_t rmsIndex = 0;
bool rmsFull = false;

// FFT circular buffers
float xFftBuf[FFT_WINDOW];
float yFftBuf[FFT_WINDOW];
float zFftBuf[FFT_WINDOW];
uint16_t fftIndex = 0;
bool fftFull = false;

// FFT work buffers
float axisWork[FFT_WINDOW];
double vReal[FFT_WINDOW];
double vImag[FFT_WINDOW];

ArduinoFFT<double> FFT(vReal, vImag, FFT_WINDOW, SAMPLE_RATE_HZ);

// Last sample
float lastXmg = 0.0f;
float lastYmg = 0.0f;
float lastZmg = 0.0f;

// Timing
uint32_t nextSampleUs = 0;
uint32_t lastSendMs = 0;

// -------------------- helpers --------------------
int toPositiveInt(float v) {
  if (v < 0) v = -v;
  long n = lroundf(v);
  if (n < 0) n = 0;
  if (n > 999999) n = 999999;
  return (int)n;
}

void witsPrint(uint16_t code, int value) {
  Serial.print(code);
  Serial.println(value);
}

void sendWITS(int x,int y,int z,int rms,int rmsAC,int freq){

  Serial.println("&&");

  Serial.print("2711");
  Serial.println(x);

  Serial.print("2712");
  Serial.println(y);

  Serial.print("2713");
  Serial.println(z);

  Serial.print("2714");
  Serial.println(rms);

  Serial.print("2715");
  Serial.println(rmsAC);

  Serial.print("2716");
  Serial.println(freq);

  Serial.println("!!");
}

// -------------------- ADXL345 low-level --------------------
bool writeRegister8(uint8_t reg, uint8_t value) {
  Wire1.beginTransmission(ADXL_ADDR);
  Wire1.write(reg);
  Wire1.write(value);
  return (Wire1.endTransmission() == 0);
}

bool readRegister8(uint8_t reg, uint8_t &value) {
  Wire1.beginTransmission(ADXL_ADDR);
  Wire1.write(reg);
  if (Wire1.endTransmission(false) != 0) return false;

  if (Wire1.requestFrom((int)ADXL_ADDR, 1) != 1) return false;
  value = Wire1.read();
  return true;
}

bool readXYZraw(int16_t &x, int16_t &y, int16_t &z) {
  Wire1.beginTransmission(ADXL_ADDR);
  Wire1.write(REG_DATAX0);
  if (Wire1.endTransmission(false) != 0) return false;

  if (Wire1.requestFrom((int)ADXL_ADDR, 6) != 6) return false;

  uint8_t x0 = Wire1.read();
  uint8_t x1 = Wire1.read();
  uint8_t y0 = Wire1.read();
  uint8_t y1 = Wire1.read();
  uint8_t z0 = Wire1.read();
  uint8_t z1 = Wire1.read();

  x = (int16_t)((x1 << 8) | x0);
  y = (int16_t)((y1 << 8) | y0);
  z = (int16_t)((z1 << 8) | z0);
  return true;
}

bool initADXL345() {
  uint8_t devid = 0;
  if (!readRegister8(REG_DEVID, devid)) return false;
  if (devid != 0xE5) return false;

  // standby
  if (!writeRegister8(REG_POWER_CTL, 0x00)) return false;

  // FULL_RES=1, range=±4g
  // 0x08 = full_res, 0x01 = ±4g
  if (!writeRegister8(REG_DATA_FORMAT, 0x09)) return false;

  // 200 Hz output data rate
  if (!writeRegister8(REG_BW_RATE, 0x0B)) return false;

  // measurement mode
  if (!writeRegister8(REG_POWER_CTL, 0x08)) return false;

  delay(10);
  return true;
}

// In FULL_RES mode ADXL345 is ~3.9 mg/LSB
inline float rawToMg(int16_t raw) {
  return raw * 3.9f;
}

// -------------------- buffer helpers --------------------
void addRmsSample(float x, float y, float z) {
  xRmsBuf[rmsIndex] = x;
  yRmsBuf[rmsIndex] = y;
  zRmsBuf[rmsIndex] = z;

  rmsIndex++;
  if (rmsIndex >= RMS_WINDOW) {
    rmsIndex = 0;
    rmsFull = true;
  }
}

void addFftSample(float x, float y, float z) {
  xFftBuf[fftIndex] = x;
  yFftBuf[fftIndex] = y;
  zFftBuf[fftIndex] = z;

  fftIndex++;
  if (fftIndex >= FFT_WINDOW) {
    fftIndex = 0;
    fftFull = true;
  }
}

void copyOrderedCircular(const float *src, float *dst, uint16_t size, uint16_t startIndex) {
  uint16_t j = 0;
  for (uint16_t i = startIndex; i < size; i++) dst[j++] = src[i];
  for (uint16_t i = 0; i < startIndex; i++)    dst[j++] = src[i];
}

// -------------------- calculations --------------------
float calcRmsTotal1s() {
  double s = 0.0;
  for (uint16_t i = 0; i < RMS_WINDOW; i++) {
    s += (double)xRmsBuf[i] * xRmsBuf[i]
       + (double)yRmsBuf[i] * yRmsBuf[i]
       + (double)zRmsBuf[i] * zRmsBuf[i];
  }
  return sqrt(s / RMS_WINDOW);
}

float calcRmsAc1s(float &meanX, float &meanY, float &meanZ,
                  float &rmsXac, float &rmsYac, float &rmsZac) {
  double sx = 0.0, sy = 0.0, sz = 0.0;
  for (uint16_t i = 0; i < RMS_WINDOW; i++) {
    sx += xRmsBuf[i];
    sy += yRmsBuf[i];
    sz += zRmsBuf[i];
  }

  meanX = sx / RMS_WINDOW;
  meanY = sy / RMS_WINDOW;
  meanZ = sz / RMS_WINDOW;

  double sxyz = 0.0;
  double sxx = 0.0, syy = 0.0, szz = 0.0;

  for (uint16_t i = 0; i < RMS_WINDOW; i++) {
    double ax = xRmsBuf[i] - meanX;
    double ay = yRmsBuf[i] - meanY;
    double az = zRmsBuf[i] - meanZ;

    sxyz += ax * ax + ay * ay + az * az;
    sxx  += ax * ax;
    syy  += ay * ay;
    szz  += az * az;
  }

  rmsXac = sqrt(sxx / RMS_WINDOW);
  rmsYac = sqrt(syy / RMS_WINDOW);
  rmsZac = sqrt(szz / RMS_WINDOW);

  return sqrt(sxyz / RMS_WINDOW);
}

float dominantFrequencyFromOrderedAxis(const float *buf) {
  double mean = 0.0;
  for (uint16_t i = 0; i < FFT_WINDOW; i++) mean += buf[i];
  mean /= FFT_WINDOW;

  for (uint16_t i = 0; i < FFT_WINDOW; i++) {
    vReal[i] = buf[i] - mean;
    vImag[i] = 0.0;
  }

  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();

  uint16_t startBin = (uint16_t)ceil((FREQ_MIN * FFT_WINDOW) / (float)SAMPLE_RATE_HZ);
  uint16_t endBin   = (uint16_t)floor((FREQ_MAX * FFT_WINDOW) / (float)SAMPLE_RATE_HZ);

  if (startBin < 1) startBin = 1;
  if (endBin >= FFT_WINDOW / 2) endBin = (FFT_WINDOW / 2) - 1;

  double maxMag = 0.0;
  uint16_t maxBin = 0;

  for (uint16_t i = startBin; i <= endBin; i++) {
    if (vReal[i] > maxMag) {
      maxMag = vReal[i];
      maxBin = i;
    }
  }

  if (maxBin == 0) return 0.0f;
  return ((float)maxBin * SAMPLE_RATE_HZ) / FFT_WINDOW;
}

// -------------------- setup / loop --------------------
void setup() {
  Serial.begin(115200);

  Wire1.begin(STEMMA_SDA, STEMMA_SCL);
  Wire1.setClock(100000);

  if (!initADXL345()) {
    while (1) {
      Serial.println("&&");
      witsPrint(2711, 0);
      witsPrint(2712, 0);
      witsPrint(2713, 0);
      witsPrint(2714, 0);
      witsPrint(2715, 0);
      witsPrint(2716, 0);
      Serial.println("!!");
      delay(1000);
    }
  }

  nextSampleUs = micros();
  lastSendMs = millis();
}

void loop() {
  // sampling at 200 Hz
  while ((int32_t)(micros() - nextSampleUs) >= 0) {
    nextSampleUs += SAMPLE_PERIOD_US;

    int16_t xr, yr, zr;
    if (readXYZraw(xr, yr, zr)) {
      float x = rawToMg(xr);
      float y = rawToMg(yr);
      float z = rawToMg(zr);

      lastXmg = x;
      lastYmg = y;
      lastZmg = z;

      addRmsSample(x, y, z);
      addFftSample(x, y, z);
    }
  }

  // send every 1 s
  if ((millis() - lastSendMs) >= 1000 && rmsFull) {
    lastSendMs += 1000;

    float meanX, meanY, meanZ;
    float rmsXac, rmsYac, rmsZac;

    float rmsAc = calcRmsAc1s(meanX, meanY, meanZ, rmsXac, rmsYac, rmsZac);
    float rmsTotal = calcRmsTotal1s();

    float freq = 0.0f;

    if (fftFull && rmsAc >= RMS_AC_THRESHOLD) {
      if (rmsXac >= rmsYac && rmsXac >= rmsZac) {
        copyOrderedCircular(xFftBuf, axisWork, FFT_WINDOW, fftIndex);
        freq = dominantFrequencyFromOrderedAxis(axisWork);
      } else if (rmsYac >= rmsXac && rmsYac >= rmsZac) {
        copyOrderedCircular(yFftBuf, axisWork, FFT_WINDOW, fftIndex);
        freq = dominantFrequencyFromOrderedAxis(axisWork);
      } else {
        copyOrderedCircular(zFftBuf, axisWork, FFT_WINDOW, fftIndex);
        freq = dominantFrequencyFromOrderedAxis(axisWork);
      }
    }

    sendWITS(
      toPositiveInt(lastXmg),
      toPositiveInt(lastYmg),
      toPositiveInt(lastZmg),
      toPositiveInt(rmsTotal),
      toPositiveInt(rmsAc),
      toPositiveInt(freq)
    );
  }
}