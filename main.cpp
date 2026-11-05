#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <WiFi.h>
#include <time.h>
#include <esp_timer.h>
#include <math.h>
#include <FirebaseESP32.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <FS.h>
#include <SD.h>
#include "secrets.h"

// ============================================================
// MONITOREO DE VIBRACIONES TRIAXIALES - ESP32 + ADXL345
//
// >>> CAMBIO FACIL ENTRE SIMULACION Y SENSOR REAL <<<
//
//   SIMULATE_SENSOR 1  -> datos aleatorios/sinteticos (Wokwi)
//   SIMULATE_SENSOR 0  -> ADXL345 real por SPI
//
// Solo cambia el numero de la linea de abajo. Nada mas.
// (En VS Code, el bloque que no esta activo se ve en gris.)
// ============================================================
#define SIMULATE_SENSOR 1

// ---------------- Pines ESP32 ----------------
// Solo se usan con el sensor real (SIMULATE_SENSOR 0)
static constexpr int PIN_SPI_SCK  = 18;
static constexpr int PIN_SPI_MISO = 19;
static constexpr int PIN_SPI_MOSI = 23;
static constexpr int PIN_ADXL_CS  = 27;
static constexpr int PIN_SD_CS    = 13;   // CS de la microSD (bus SPI compartido)

// OLED por I2C
static constexpr int PIN_I2C_SDA = 21;
static constexpr int PIN_I2C_SCL = 22;

// ---------------- OLED ----------------
static constexpr uint8_t OLED_ADDR = 0x3C;
static constexpr int SCREEN_WIDTH = 128;
static constexpr int SCREEN_HEIGHT = 64;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
bool oledOK = false;

// ---------------- ADXL345 (registros) ----------------
static constexpr uint8_t ADXL_DEVID       = 0x00;
static constexpr uint8_t ADXL_BW_RATE     = 0x2C;
static constexpr uint8_t ADXL_POWER_CTL   = 0x2D;
static constexpr uint8_t ADXL_DATA_FORMAT = 0x31;
static constexpr uint8_t ADXL_DATAX0      = 0x32;
static constexpr uint8_t ADXL_FIFO_CTL    = 0x38;
static constexpr float ADXL_SCALE_G_LSB   = 0.0039f; // 3.9 mg/LSB en FULL_RES
static constexpr float GRAVITY            = 9.80665f;
static constexpr uint8_t ADXL_EXPECTED_ID = 0xE5;
bool adxlOK = false;

// ---------------- Muestreo y procesamiento ----------------
static constexpr uint16_t SAMPLE_RATE_HZ = 800;
static constexpr uint16_t WINDOW_SIZE = 512; // potencia de 2
static constexpr uint32_t SAMPLE_PERIOD_US = 1000000UL / SAMPLE_RATE_HZ; // 1250 us
static constexpr float HP_CUTOFF_HZ = 0.5f;
static constexpr float HP_DT = 1.0f / SAMPLE_RATE_HZ;
static constexpr float HP_RC = 1.0f / (2.0f * PI * HP_CUTOFF_HZ);
static constexpr float HP_ALPHA = HP_RC / (HP_RC + HP_DT);

float offsetX_g = 0.0f;
float offsetY_g = 0.0f;
float offsetZ_g = 0.0f;

struct VibrationResult {
  float rmsX = 0.0f;
  float rmsY = 0.0f;
  float rmsZ = 0.0f;
  float peakX = 0.0f;   // frecuencia pico [Hz]
  float peakY = 0.0f;
  float peakZ = 0.0f;
  uint32_t sampleCount = 0;
  uint32_t timestamp = 0;
};

// Doble buffer: mientras una ventana se procesa, la otra se llena.
static float bufX[2][WINDOW_SIZE];
static float bufY[2][WINDOW_SIZE];
static float bufZ[2][WINDOW_SIZE];

QueueHandle_t windowQueue = nullptr;  // indice (0/1) del buffer lleno
QueueHandle_t resultQueue = nullptr;  // ultimo resultado para loop()
VibrationResult latestResult;
static TaskHandle_t acqTaskHandle = nullptr;

// ---------------- microSD ----------------
// El ADXL345 y la microSD comparten el bus SPI. El mutex evita que las dos
// tareas (nucleo 0 escribe la SD, nucleo 1 lee el sensor) usen el bus a la vez.
SemaphoreHandle_t spiMutex = nullptr;
File logFile;
bool sdOK = false;
uint32_t windowCounter = 0;

// Estado de los filtros pasa-altos (uno por eje).
float hpPrevInput[3]  = {0.0f, 0.0f, 0.0f};
float hpPrevOutput[3] = {0.0f, 0.0f, 0.0f};

// ---------------- Firebase ----------------
FirebaseConfig firebaseConfig;
FirebaseAuth firebaseAuth;
FirebaseData firebaseData;
bool firebaseEnabled = false;

static constexpr uint32_t LAST_RECORD_PERIOD_MS = 2000;
static constexpr uint32_t HISTORY_PERIOD_MS = 10000;
static constexpr uint32_t OLED_PERIOD_MS = 250;
uint32_t lastRecordMs = 0;
uint32_t lastHistoryMs = 0;
uint32_t lastOledMs = 0;
uint32_t lastWifiAttemptMs = 0;

// ============================================================
// ============  SENSOR REAL: ADXL345 por SPI  ================
// (activo solo con SIMULATE_SENSOR 0)
// ============================================================
#if !SIMULATE_SENSOR

uint8_t adxlReadRegister(uint8_t reg) {
  SPI.beginTransaction(SPISettings(5000000, MSBFIRST, SPI_MODE3));
  digitalWrite(PIN_ADXL_CS, LOW);
  SPI.transfer(reg | 0x80); // bit 7 = lectura
  uint8_t value = SPI.transfer(0x00);
  digitalWrite(PIN_ADXL_CS, HIGH);
  SPI.endTransaction();
  return value;
}

void adxlWriteRegister(uint8_t reg, uint8_t value) {
  SPI.beginTransaction(SPISettings(5000000, MSBFIRST, SPI_MODE3));
  digitalWrite(PIN_ADXL_CS, LOW);
  SPI.transfer(reg & 0x3F); // escritura de un registro
  SPI.transfer(value);
  digitalWrite(PIN_ADXL_CS, HIGH);
  SPI.endTransaction();
}

bool adxlReadXYZ(int16_t &x, int16_t &y, int16_t &z) {
  uint8_t data[6];
  SPI.beginTransaction(SPISettings(5000000, MSBFIRST, SPI_MODE3));
  digitalWrite(PIN_ADXL_CS, LOW);
  SPI.transfer(0x80 | 0x40 | ADXL_DATAX0); // lectura + multibyte
  for (int i = 0; i < 6; i++) data[i] = SPI.transfer(0x00);
  digitalWrite(PIN_ADXL_CS, HIGH);
  SPI.endTransaction();

  x = (int16_t)((uint16_t)data[1] << 8 | data[0]);
  y = (int16_t)((uint16_t)data[3] << 8 | data[2]);
  z = (int16_t)((uint16_t)data[5] << 8 | data[4]);
  return true;
}

bool initADXL345() {
  pinMode(PIN_ADXL_CS, OUTPUT);
  digitalWrite(PIN_ADXL_CS, HIGH);
  // El bus SPI se inicia en setup() (compartido con la microSD).
  delay(20);

  uint8_t id = adxlReadRegister(ADXL_DEVID);
  Serial.printf("ADXL345 DEVID = 0x%02X\n", id);
  if (id != ADXL_EXPECTED_ID) return false;

  adxlWriteRegister(ADXL_POWER_CTL, 0x00);   // standby para configurar
  adxlWriteRegister(ADXL_BW_RATE, 0x0D);     // ODR = 800 Hz
  adxlWriteRegister(ADXL_DATA_FORMAT, 0x0A); // FULL_RES + rango +/-8 g
  adxlWriteRegister(ADXL_FIFO_CTL, 0x00);    // FIFO bypass
  adxlWriteRegister(ADXL_POWER_CTL, 0x08);   // modo medicion
  delay(20);
  return true;
}

bool readSensorG(float &xg, float &yg, float &zg) {
  int16_t rx, ry, rz;
  if (!adxlReadXYZ(rx, ry, rz)) return false;
  xg = rx * ADXL_SCALE_G_LSB;
  yg = ry * ADXL_SCALE_G_LSB;
  zg = rz * ADXL_SCALE_G_LSB;
  return true;
}

#endif // !SIMULATE_SENSOR

// ============================================================
// ============  SENSOR SIMULADO: datos aleatorios  ===========
// (activo solo con SIMULATE_SENSOR 1)
// ============================================================
#if SIMULATE_SENSOR

// Ruido aleatorio uniforme en [-amp, +amp]
static float ruidoAleatorio(float amp) {
  return (((float)(esp_random() % 2001) - 1000.0f) / 1000.0f) * amp;
}

void initSimulatedSensor() {
  adxlOK = true;
  Serial.println("MODO SIMULACION: datos aleatorios activados");
}

// Numero aleatorio uniforme en [a, b]
static float aleatorioEntre(float a, float b) {
  return a + (b - a) * ((float)(esp_random() % 10001) / 10000.0f);
}

// Cada eje tiene 2 tonos. Su frecuencia y amplitud derivan lentamente hacia
// un objetivo aleatorio que se renueva cada 2-6 s (como cambia la velocidad,
// el regimen del motor o el tipo de camino en un vehiculo real).
struct SimTone {
  float freq, freqTarget, fMin, fMax;   // frecuencia actual/objetivo y rango [Hz]
  float amp, ampTarget, aMin, aMax;     // amplitud actual/objetivo y rango [g]
  float phase;                          // fase acumulada (evita saltos)
};

static SimTone simTones[3][2] = {
  // X (longitudinal): motor/transmision
  {{25.0f, 25.0f, 15.0f, 45.0f, 0.050f, 0.050f, 0.030f, 0.080f, 0.0f},
   {10.0f, 10.0f,  5.0f, 15.0f, 0.020f, 0.020f, 0.010f, 0.030f, 0.0f}},
  // Y (lateral): balanceo y curvas
  {{10.0f, 10.0f,  3.0f, 20.0f, 0.035f, 0.035f, 0.020f, 0.050f, 0.0f},
   {30.0f, 30.0f, 20.0f, 40.0f, 0.010f, 0.010f, 0.005f, 0.015f, 0.0f}},
  // Z (vertical): suspension/ruedas + rebote de carroceria
  {{15.0f, 15.0f,  8.0f, 25.0f, 0.070f, 0.070f, 0.040f, 0.100f, 0.0f},
   { 2.5f,  2.5f,  1.0f,  4.0f, 0.035f, 0.035f, 0.020f, 0.050f, 0.0f}}
};

static uint32_t simRetargetCountdown = 0;
static uint32_t simBumpCountdown = 3UL * SAMPLE_RATE_HZ;  // primer bache a los ~3 s
static float simBumpAmp = 0.0f, simBumpFreq = 15.0f, simBumpPhase = 0.0f;

static void simNuevosObjetivos() {
  for (int a = 0; a < 3; a++) {
    for (int k = 0; k < 2; k++) {
      SimTone &s = simTones[a][k];
      s.freqTarget = aleatorioEntre(s.fMin, s.fMax);
      s.ampTarget  = aleatorioEntre(s.aMin, s.aMax);
    }
  }
  simRetargetCountdown = (uint32_t)(aleatorioEntre(2.0f, 6.0f) * SAMPLE_RATE_HZ);
}

static float simTonosEje(int axis) {
  float suma = 0.0f;
  for (int k = 0; k < 2; k++) {
    SimTone &s = simTones[axis][k];
    s.freq += (s.freqTarget - s.freq) * 0.001f;   // deriva suave (~1.5 s)
    s.amp  += (s.ampTarget  - s.amp)  * 0.001f;
    s.phase += 2.0f * PI * s.freq / SAMPLE_RATE_HZ;
    if (s.phase > 2.0f * PI) s.phase -= 2.0f * PI;
    suma += s.amp * sinf(s.phase);
  }
  return suma;
}

bool readSensorG(float &xg, float &yg, float &zg) {
  // Nuevos objetivos de frecuencia/amplitud cada 2-6 s
  if (simRetargetCountdown == 0) simNuevosObjetivos();
  else simRetargetCountdown--;

  // Baches: cada 4-12 s un golpe corto que decae (~0.25 s)
  float bache = 0.0f;
  if (simBumpCountdown == 0) {
    simBumpAmp = aleatorioEntre(0.20f, 0.60f);
    simBumpFreq = aleatorioEntre(10.0f, 20.0f);
    simBumpPhase = 0.0f;
    simBumpCountdown = (uint32_t)(aleatorioEntre(4.0f, 12.0f) * SAMPLE_RATE_HZ);
  } else {
    simBumpCountdown--;
  }
  if (simBumpAmp > 0.001f) {
    simBumpPhase += 2.0f * PI * simBumpFreq / SAMPLE_RATE_HZ;
    bache = simBumpAmp * sinf(simBumpPhase);
    simBumpAmp *= 0.995f;
  }

  xg = simTonosEje(0) + 0.3f * bache + ruidoAleatorio(0.010f);
  yg = simTonosEje(1) + 0.2f * bache + ruidoAleatorio(0.010f);
  zg = 1.0f + simTonosEje(2) + bache + ruidoAleatorio(0.010f);
  return true;
}

#endif // SIMULATE_SENSOR

// ============================================================
// Calibracion estatica (sensor inmovil, eje Z hacia arriba)
// ============================================================
void calibrateAccelerometer() {
#if SIMULATE_SENSOR
  offsetX_g = 0.0f;
  offsetY_g = 0.0f;
  offsetZ_g = 0.0f;
  Serial.println("Calibracion simulada: offsets en cero");
#else
  const int CAL_SAMPLES = 256;
  float sx = 0.0f, sy = 0.0f, sz = 0.0f;
  Serial.println("Calibrando: mantener el sensor inmovil y con Z hacia arriba...");
  for (int i = 0; i < CAL_SAMPLES; i++) {
    float xg, yg, zg;
    if (readSensorG(xg, yg, zg)) {
      sx += xg; sy += yg; sz += zg;
    }
    delayMicroseconds(SAMPLE_PERIOD_US);
  }
  offsetX_g = sx / CAL_SAMPLES;
  offsetY_g = sy / CAL_SAMPLES;
  offsetZ_g = (sz / CAL_SAMPLES) - 1.0f;
  Serial.printf("Offsets [g]: X=%.5f Y=%.5f Z=%.5f\n", offsetX_g, offsetY_g, offsetZ_g);
#endif
}

// ============================================================
// Filtro pasa-altos de primer orden (quita gravedad y deriva)
// ============================================================
float highPassFilter(float input, int axis) {
  float output = HP_ALPHA * (hpPrevOutput[axis] + input - hpPrevInput[axis]);
  hpPrevInput[axis] = input;
  hpPrevOutput[axis] = output;
  return output;
}

// ============================================================
// FFT radix-2 in-place. Devuelve la frecuencia del pico (sin DC).
// Solo la llama processingTask, por eso los arreglos static son seguros.
// ============================================================
float calculatePeakFrequency(const float *samples) {
  static float realPart[WINDOW_SIZE];
  static float imagPart[WINDOW_SIZE];

  for (uint16_t i = 0; i < WINDOW_SIZE; i++) {
    float window = 0.54f - 0.46f * cosf((2.0f * PI * i) / (WINDOW_SIZE - 1));
    realPart[i] = samples[i] * window;
    imagPart[i] = 0.0f;
  }

  // Inversion de bits
  uint16_t j = 0;
  for (uint16_t i = 1; i < WINDOW_SIZE; i++) {
    uint16_t bit = WINDOW_SIZE >> 1;
    while (j & bit) { j ^= bit; bit >>= 1; }
    j ^= bit;
    if (i < j) {
      float tr = realPart[i]; realPart[i] = realPart[j]; realPart[j] = tr;
      float ti = imagPart[i]; imagPart[i] = imagPart[j]; imagPart[j] = ti;
    }
  }

  // Mariposas
  for (uint16_t len = 2; len <= WINDOW_SIZE; len <<= 1) {
    float angle = -2.0f * PI / len;
    float wLenR = cosf(angle);
    float wLenI = sinf(angle);
    for (uint16_t i = 0; i < WINDOW_SIZE; i += len) {
      float wr = 1.0f, wi = 0.0f;
      for (uint16_t k = 0; k < len / 2; k++) {
        uint16_t even = i + k;
        uint16_t odd = even + len / 2;
        float tr = wr * realPart[odd] - wi * imagPart[odd];
        float ti = wr * imagPart[odd] + wi * realPart[odd];
        realPart[odd] = realPart[even] - tr;
        imagPart[odd] = imagPart[even] - ti;
        realPart[even] += tr;
        imagPart[even] += ti;
        float nextWr = wr * wLenR - wi * wLenI;
        wi = wr * wLenI + wi * wLenR;
        wr = nextWr;
      }
    }
  }

  float maxMagnitude = 0.0f;
  uint16_t maxIndex = 0;
  for (uint16_t k = 1; k < WINDOW_SIZE / 2; k++) {
    float mag = realPart[k] * realPart[k] + imagPart[k] * imagPart[k];
    if (mag > maxMagnitude) { maxMagnitude = mag; maxIndex = k; }
  }
  return (float)maxIndex * SAMPLE_RATE_HZ / WINDOW_SIZE;
}

// ============================================================
// microSD: registro CSV de cada ventana (cada 0.64 s)
// ============================================================
bool initSD() {
  if (!SD.begin(PIN_SD_CS, SPI, 4000000)) {
    Serial.println("microSD: no se pudo inicializar (revisar cableado y formato FAT32).");
    return false;
  }

  // Archivo nuevo en cada arranque: vib_001.csv, vib_002.csv, ...
  char name[24];
  for (int i = 1; i < 1000; i++) {
    snprintf(name, sizeof(name), "/vib_%03d.csv", i);
    if (!SD.exists(name)) break;
  }

  logFile = SD.open(name, FILE_WRITE);
  if (!logFile) {
    Serial.println("microSD: no se pudo crear el archivo.");
    return false;
  }
  logFile.println("ventana,millis,epoch,rms_x,rms_y,rms_z,pico_x_hz,pico_y_hz,pico_z_hz");
  logFile.flush();
  Serial.printf("microSD: registrando en %s\n", name);
  return true;
}

void logWindowToSD(const VibrationResult &r) {
  if (!sdOK || !spiMutex) return;

  char line[128];
  snprintf(line, sizeof(line), "%lu,%lu,%lu,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f",
           (unsigned long)(++windowCounter), (unsigned long)millis(),
           (unsigned long)r.timestamp,
           r.rmsX, r.rmsY, r.rmsZ, r.peakX, r.peakY, r.peakZ);

  xSemaphoreTake(spiMutex, portMAX_DELAY);
  logFile.println(line);
  logFile.flush();   // guarda ya: si se apaga el ESP32 no se pierde la ventana
  xSemaphoreGive(spiMutex);
}

// ============================================================
// Procesa una ventana: RMS + frecuencia pico por eje
// ============================================================
void processWindow(const float *x, const float *y, const float *z) {
  VibrationResult result;
  double sumX = 0.0, sumY = 0.0, sumZ = 0.0;
  for (uint16_t i = 0; i < WINDOW_SIZE; i++) {
    sumX += (double)x[i] * x[i];
    sumY += (double)y[i] * y[i];
    sumZ += (double)z[i] * z[i];
  }
  result.rmsX = sqrt(sumX / WINDOW_SIZE);
  result.rmsY = sqrt(sumY / WINDOW_SIZE);
  result.rmsZ = sqrt(sumZ / WINDOW_SIZE);
  result.peakX = calculatePeakFrequency(x);
  result.peakY = calculatePeakFrequency(y);
  result.peakZ = calculatePeakFrequency(z);
  result.sampleCount = WINDOW_SIZE;
  result.timestamp = (uint32_t)time(nullptr);

  if (resultQueue) xQueueOverwrite(resultQueue, &result);
  logWindowToSD(result);   // guarda TODAS las ventanas (resolucion 0.64 s)
}

// ============================================================
// Timer: despierta la tarea de adquisicion cada 1250 us (800 Hz)
// ============================================================
void onSampleTimer(void *arg) {
  if (acqTaskHandle) xTaskNotifyGive(acqTaskHandle);
}

// ============================================================
// Tarea de adquisicion (nucleo 1, prioridad alta).
// DUERME entre muestras: asi setup() y loop() siguen corriendo.
// ============================================================
void acquisitionTask(void *parameter) {
  uint16_t index = 0;
  uint8_t active = 0;

  while (true) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // espera al timer

    float xg = 0.0f, yg = 0.0f, zg = 1.0f;
    // Bus SPI compartido: tomar el mutex solo durante la lectura (~15 us).
    if (spiMutex) xSemaphoreTake(spiMutex, portMAX_DELAY);
    bool lecturaOK = readSensorG(xg, yg, zg);
    if (spiMutex) xSemaphoreGive(spiMutex);
    if (!lecturaOK) continue;

    // g -> m/s2 y correccion de offsets
    float ax = (xg - offsetX_g) * GRAVITY;
    float ay = (yg - offsetY_g) * GRAVITY;
    float az = (zg - offsetZ_g) * GRAVITY - GRAVITY; // quita 1 g de Z

    bufX[active][index] = highPassFilter(ax, 0);
    bufY[active][index] = highPassFilter(ay, 1);
    bufZ[active][index] = highPassFilter(az, 2);
    index++;

    if (index >= WINDOW_SIZE) {
      xQueueSend(windowQueue, &active, 0); // avisa: ventana llena
      active ^= 1;                         // cambia al otro buffer
      index = 0;
    }
  }
}

// ============================================================
// Tarea de procesamiento (nucleo 0, prioridad baja): hace las FFT
// sin quitarle tiempo al muestreo.
// ============================================================
void processingTask(void *parameter) {
  uint8_t full;
  while (true) {
    if (xQueueReceive(windowQueue, &full, portMAX_DELAY) == pdTRUE) {
      processWindow(bufX[full], bufY[full], bufZ[full]);
    }
  }
}

// ============================================================
// Wi-Fi y Firebase
// ============================================================
bool hasFirebaseCredentials() {
  String host = FIREBASE_HOST;
  String token = FIREBASE_LEGACY_TOKEN;
  return !host.startsWith("TU-") && !token.startsWith("REEMPLAZAR");
}

void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  Serial.printf("Conectando a Wi-Fi: %s\n", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 12000) {
    delay(300);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\nWi-Fi conectado. IP: %s\n", WiFi.localIP().toString().c_str());
    configTime(0, 0, "pool.ntp.org", "time.google.com");
  } else {
    Serial.println("\nNo se pudo conectar a Wi-Fi. Se reintentara desde loop().");
  }
}

void initFirebase() {
  if (!hasFirebaseCredentials()) {
    Serial.println("Firebase pendiente: complete include/secrets.h");
    firebaseEnabled = false;
    return;
  }
  firebaseConfig.database_url = FIREBASE_HOST;
  firebaseConfig.signer.tokens.legacy_token = FIREBASE_LEGACY_TOKEN;
  Firebase.begin(&firebaseConfig, &firebaseAuth);
  Firebase.reconnectWiFi(true);
  firebaseEnabled = true;
  Serial.println("Cliente Firebase inicializado.");
}

void fillFirebaseJson(FirebaseJson &json, const VibrationResult &r) {
  json.set("nombre", VEHICLE_NAME);
  json.set("rms_x_m_s2", r.rmsX);
  json.set("rms_y_m_s2", r.rmsY);
  json.set("rms_z_m_s2", r.rmsZ);
  json.set("frecuencia_pico_x_hz", r.peakX);
  json.set("frecuencia_pico_y_hz", r.peakY);
  json.set("frecuencia_pico_z_hz", r.peakZ);
  json.set("frecuencia_muestreo_hz", SAMPLE_RATE_HZ);
  json.set("muestras_ventana", r.sampleCount);
  json.set("timestamp", (int)r.timestamp);
  json.set("modo_sensor", SIMULATE_SENSOR ? "simulado" : "ADXL345_SPI");
}

void sendLastRecord(const VibrationResult &r) {
  if (!firebaseEnabled || WiFi.status() != WL_CONNECTED || !Firebase.ready()) return;
  FirebaseJson json;
  fillFirebaseJson(json, r);
  String path = String("/vibraciones/") + VEHICLE_ID + "/ultimo_registro";
  if (Firebase.setJSON(firebaseData, path, json)) {
    Serial.println("Firebase ultimo_registro: OK");
  } else {
    Serial.printf("Firebase error: %s\n", firebaseData.errorReason().c_str());
  }
}

void sendHistory(const VibrationResult &r) {
  if (!firebaseEnabled || WiFi.status() != WL_CONNECTED || !Firebase.ready()) return;
  FirebaseJson json;
  fillFirebaseJson(json, r);
  String path = String("/vibraciones/") + VEHICLE_ID + "/historial";
  if (Firebase.pushJSON(firebaseData, path, json)) {
    Serial.printf("Firebase historial: OK (%s)\n", firebaseData.pushName().c_str());
  } else {
    Serial.printf("Firebase historial error: %s\n", firebaseData.errorReason().c_str());
  }
}

// ============================================================
// OLED
// ============================================================
void updateOLED(const VibrationResult &r) {
  if (!oledOK) return;
  if (millis() - lastOledMs < OLED_PERIOD_MS) return;
  lastOledMs = millis();

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("VIBRACION TRIAXIAL");
  display.drawFastHLine(0, 10, 128, SSD1306_WHITE);

  display.setCursor(0, 14);
  display.printf("RMS X: %6.3f m/s2", r.rmsX);
  display.setCursor(0, 25);
  display.printf("RMS Y: %6.3f m/s2", r.rmsY);
  display.setCursor(0, 36);
  display.printf("RMS Z: %6.3f m/s2", r.rmsZ);
  display.drawFastHLine(0, 47, 128, SSD1306_WHITE);
  display.setCursor(0, 51);
  display.printf("Pico X:%5.1f Hz %s", r.peakX, sdOK ? "SD" : "--");
  display.display();
}

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(400);
  Serial.println("\n=== MONITOREO DE VIBRACIONES ADXL345 ===");

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  oledOK = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  if (oledOK) {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(8, 20);
    display.println("ADXL345 + ESP32");
    display.setCursor(8, 36);
    display.println("Iniciando...");
    display.display();
  } else {
    Serial.println("OLED no detectada; se continua por Serial.");
  }

  // ---- Bus SPI compartido (ADXL345 + microSD) ----
  // Ambos CS en alto antes de iniciar, para que ninguno escuche al otro.
  pinMode(PIN_ADXL_CS, OUTPUT); digitalWrite(PIN_ADXL_CS, HIGH);
  pinMode(PIN_SD_CS, OUTPUT);   digitalWrite(PIN_SD_CS, HIGH);
  SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI);
  spiMutex = xSemaphoreCreateMutex();

  // ---- Sensor ----
#if SIMULATE_SENSOR
  initSimulatedSensor();
#else
  adxlOK = initADXL345();
  if (!adxlOK) {
    Serial.println("ERROR: no se detecta ADXL345. Revisar SPI y alimentacion.");
    while (true) delay(1000);
  }
#endif

  calibrateAccelerometer();
  connectWiFi();
  initFirebase();

  sdOK = initSD();
  if (!sdOK) Serial.println("Se continua SIN microSD (solo OLED/Serial/Firebase).");

  // ---- Colas ----
  resultQueue = xQueueCreate(1, sizeof(VibrationResult));
  windowQueue = xQueueCreate(2, sizeof(uint8_t));
  if (resultQueue == nullptr || windowQueue == nullptr) {
    Serial.println("ERROR: no se pudieron crear las colas.");
    while (true) delay(1000);
  }

  // ---- Tarea de procesamiento (FFT) en nucleo 0 ----
  if (xTaskCreatePinnedToCore(processingTask, "Proc_FFT", 8192, nullptr, 1,
                              nullptr, 0) != pdPASS) {
    Serial.println("ERROR: no se pudo crear la tarea de procesamiento.");
    while (true) delay(1000);
  }

  // ---- Tarea de adquisicion en nucleo 1 ----
  if (xTaskCreatePinnedToCore(acquisitionTask, "ADXL_Acquisition", 8192, nullptr, 3,
                              &acqTaskHandle, 1) != pdPASS) {
    Serial.println("ERROR: no se pudo crear la tarea de adquisicion.");
    while (true) delay(1000);
  }

  // ---- Timer periodico a 800 Hz que dispara cada muestra ----
  esp_timer_handle_t sampleTimer = nullptr;
  esp_timer_create_args_t timerArgs = {};
  timerArgs.callback = &onSampleTimer;
  timerArgs.arg = nullptr;
  timerArgs.dispatch_method = ESP_TIMER_TASK;
  timerArgs.name = "sample_timer";
  if (esp_timer_create(&timerArgs, &sampleTimer) != ESP_OK ||
      esp_timer_start_periodic(sampleTimer, SAMPLE_PERIOD_US) != ESP_OK) {
    Serial.println("ERROR: no se pudo iniciar el timer de muestreo.");
    while (true) delay(1000);
  }

  Serial.printf("Muestreo configurado: %u Hz | Ventana: %u muestras | Duracion: %.3f s\n",
                SAMPLE_RATE_HZ, WINDOW_SIZE,
                (float)WINDOW_SIZE / SAMPLE_RATE_HZ);
  Serial.println("Adquisicion iniciada.");
}

// ============================================================
// LOOP: serial, OLED, Wi-Fi y Firebase
// ============================================================
void loop() {
  VibrationResult newResult;
  if (resultQueue && xQueueReceive(resultQueue, &newResult, 0) == pdTRUE) {
    latestResult = newResult;
    Serial.printf("RMS [m/s2] X=%.3f Y=%.3f Z=%.3f | Pico [Hz] X=%.1f Y=%.1f Z=%.1f\n",
                  latestResult.rmsX, latestResult.rmsY, latestResult.rmsZ,
                  latestResult.peakX, latestResult.peakY, latestResult.peakZ);
  }

  updateOLED(latestResult);

  if (WiFi.status() != WL_CONNECTED && millis() - lastWifiAttemptMs > 10000) {
    lastWifiAttemptMs = millis();
    connectWiFi();
  }

  uint32_t now = millis();
  if (now - lastRecordMs >= LAST_RECORD_PERIOD_MS) {
    lastRecordMs = now;
    sendLastRecord(latestResult);
  }
  if (now - lastHistoryMs >= HISTORY_PERIOD_MS) {
    lastHistoryMs = now;
    sendHistory(latestResult);
  }

  delay(2);
}
