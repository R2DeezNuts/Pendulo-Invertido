#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <EEPROM.h>
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

// --------- Pines Motores ------------
const int pinSTBY      = 4;
const int pinPWMA      = 26; // DERECHO
const int pinAIN1      = 2;  // DERECHO
const int pinAIN2      = 0;  // DERECHO
const int pinPWMB      = 25; // IZQUIERDO
const int pinBIN1      = 16; // IZQUIERDO
const int pinBIN2      = 17; // IZQUIERDO

// --------- Variables MPU y filtro ---------
MPU6050 mpu;
int16_t ax, ay, az, gx, gy, gz;
float ang_x = 0, ang_x_prev = 0, accel_ang_x = 0;
float dt = 0;
unsigned long calibrateTime = 0;
float ANGLE_OFFSET = 0;
const float alpha = 0.995;

// --------- PID ---------
double setpoint = 1.0;
double output = 0.0;
double Kp = 20.0, Ki = 0.3, Kd = 0.3;
double integral = 0.0, derivative = 0.0, previous_error = 0.0;

// --------- Anti-windup PID ---------
const double integral_max = 200;
const double integral_min = -200;

// --------- Seguridad (caída, autoparado) ---------
const float MAX_ANGLE = 30.0; // grados
const unsigned long FALL_TIMEOUT = 1000; // ms
unsigned long fallStartTime = 0;
bool fallen = false;

// --------- Control Task ---------
const unsigned long CONTROL_PERIOD_MS = 10; // 10ms = 100Hz
unsigned long lastControlTime = 0;

// --------- Estado motores ---------
bool motorsEnabled = false;

// --------- WiFi portal ---------
const char* ssid     = "RobotAP";
const char* password = "12345678";
IPAddress apIP(192, 168, 4, 1);
IPAddress netMsk(255, 255, 255, 0);
WebServer server(80);
DNSServer dnsServer;
const byte DNS_PORT = 53;

float loopFrequency = 0.0;
float ajuste_fino = 0.0;

// --------- EEPROM ---------
#define EEPROM_SIZE 32
#define ADDR_KP      0
#define ADDR_KI      4
#define ADDR_KD      8
#define ADDR_SETPT  12
#define ADDR_AJUSTE 16
#define ADDR_MAGIC  20
#define MAGIC_VALUE 0xAA55AA55

void saveParams() {
  EEPROM.put(ADDR_KP,      Kp);
  EEPROM.put(ADDR_KI,      Ki);
  EEPROM.put(ADDR_KD,      Kd);
  EEPROM.put(ADDR_SETPT,   setpoint);
  EEPROM.put(ADDR_AJUSTE,  ajuste_fino);
  EEPROM.put(ADDR_MAGIC,   MAGIC_VALUE);
  EEPROM.commit();
}

void loadParams() {
  uint32_t magic;
  EEPROM.get(ADDR_MAGIC, magic);
  if (magic == MAGIC_VALUE) {
    EEPROM.get(ADDR_KP,      Kp);
    EEPROM.get(ADDR_KI,      Ki);
    EEPROM.get(ADDR_KD,      Kd);
    EEPROM.get(ADDR_SETPT,   setpoint);
    EEPROM.get(ADDR_AJUSTE,  ajuste_fino);
    Serial.println("Parámetros PID cargados de EEPROM.");
  } else {
    Serial.println("EEPROM no inicializada, usando valores por defecto.");
  }
}

// --------- Prototipos ---------
void handleRoot();
void handleToggle();
void handleStatus();
void handleGetSettings();
void handleSetSettings();
void handleNotFound();
void handleCalibrate();
void setupWiFiPortal();
void driveMotors(int value);
void ControlTask(void * parameter);

// --------- Motores ------------
void driveMotors(int value) {
  value = constrain(value, -255, 255);
  int mag = abs(value);

  // Motor IZQUIERDO (PWMB, BIN1, BIN2)
  if (value > 0) {
    digitalWrite(pinBIN1, HIGH); digitalWrite(pinBIN2, LOW); // Adelante
  } else if (value < 0) {
    digitalWrite(pinBIN1, LOW); digitalWrite(pinBIN2, HIGH); // Atrás
  } else {
    digitalWrite(pinBIN1, HIGH); digitalWrite(pinBIN2, HIGH);
    mag = 0;
  }
  analogWrite(pinPWMB, mag);

  // Motor DERECHO (PWMA, AIN1, AIN2)
  if (value > 0) {
    digitalWrite(pinAIN1, HIGH); digitalWrite(pinAIN2, LOW); // Adelante
  } else if (value < 0) {
    digitalWrite(pinAIN1, LOW); digitalWrite(pinAIN2, HIGH); // Atrás
  } else {
    digitalWrite(pinAIN1, HIGH); digitalWrite(pinAIN2, HIGH);
    mag = 0;
  }
  analogWrite(pinPWMA, mag);
}

// --------- Handlers web ---------
void handleRoot() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { margin:0; display:flex; flex-direction:column; justify-content:center; align-items:center; height:100vh; background:#000; color:#fff; }
    button, input { font-size:1.2em; margin:5px; }
    button { padding:10px; border:none; border-radius:5px; width:80%; height:40px; }
    .on { background:#4CAF50; color:#fff; }
    .off { background:#F44336; color:#fff; }
    #controls { margin-top:20px; display:flex; flex-direction:column; align-items:center; }
    label { display:flex; justify-content:space-between; width:80%; }
    input { width:60px; }
  </style>
</head>
<body>
  <button id="btn" class="off">OFF</button>
  <button id="calibrate">Calibrar ángulo cero</button>
  <div id="status">Cargando...</div>
  <div id="controls">
    <label>Kp: <input id="Kp" type="number" step="0.1"></label>
    <label>Ki: <input id="Ki" type="number" step="0.01"></label>
    <label>Kd: <input id="Kd" type="number" step="0.1"></label>
    <label>Setpoint: <input id="Setpoint" type="number" step="0.1"></label>
    <label>Ajuste fino: <input id="Ajuste" type="number" step="0.01"></label>
    <button id="apply">Aplicar</button>
  </div>
  <script>
    const btn = document.getElementById('btn');
    const status = document.getElementById('status');
    const inpKp = document.getElementById('Kp');
    const inpKi = document.getElementById('Ki');
    const inpKd = document.getElementById('Kd');
    const inpSetpoint = document.getElementById('Setpoint');
    const inpAjuste = document.getElementById('Ajuste');
    const calibrate = document.getElementById('calibrate');

    function updateButton(s) { btn.textContent = s; btn.className = s.toLowerCase(); }
    btn.onclick = () => fetch('/toggle').then(r => r.text()).then(updateButton);

    calibrate.onclick = () => { fetch('/calibrate').then(r => r.text()).then(alert); };

    function updateStatus() {
      fetch('/status').then(r => r.text()).then(t => status.textContent = t);
    }
    setInterval(updateStatus, 500); updateStatus();

    fetch('/getsettings').then(r => r.json()).then(j => {
      inpKp.value = j.Kp;
      inpKi.value = j.Ki;
      inpKd.value = j.Kd;
      inpSetpoint.value = j.Setpoint;
      inpAjuste.value = j.Ajuste;
    });

    document.getElementById('apply').onclick = () => {
      const p = new URLSearchParams({
        Kp: inpKp.value,
        Ki: inpKi.value,
        Kd: inpKd.value,
        Setpoint: inpSetpoint.value,
        Ajuste: inpAjuste.value
      });
      fetch('/setsettings?' + p).then(r => r.text()).then(alert);
    };
  </script>
</body>
</html>
)rawliteral";
  server.send(200, "text/html", html);
}

void handleToggle() {
  bool prevMotorsEnabled = motorsEnabled;
  motorsEnabled = !motorsEnabled;
  // Si se acaba de activar, resetea PID:
  if (motorsEnabled && !prevMotorsEnabled) {
    integral = 0;
    derivative = 0;
    previous_error = 0;
  }
  server.send(200, "text/plain", motorsEnabled?"ON":"OFF");
}
void handleStatus() {
  String stat = String(motorsEnabled ? "ON" : "OFF") +
                (fallen ? " | CAIDO" : "") +
                " | Ángulo: " + String(ang_x, 2) +
                "° | Velocidad: " + String(abs(output), 0) +
                " | Loop: " + String(loopFrequency, 1) + " Hz";
  server.send(200, "text/plain", stat);
}
void handleGetSettings() {
  String json = String("{\"Kp\":") + String(Kp,2) + ",\"Ki\":" + String(Ki,2) +
                ",\"Kd\":" + String(Kd,2) + ",\"Setpoint\":" + String(setpoint,2) +
                ",\"Ajuste\":" + String(ajuste_fino,2) + "}";
  server.send(200, "application/json", json);
}
void handleSetSettings() {
  if (server.hasArg("Kp")) Kp = server.arg("Kp").toFloat();
  if (server.hasArg("Ki")) Ki = server.arg("Ki").toFloat();
  if (server.hasArg("Kd")) Kd = server.arg("Kd").toFloat();
  if (server.hasArg("Setpoint")) setpoint = server.arg("Setpoint").toFloat();
  if (server.hasArg("Ajuste")) ajuste_fino = server.arg("Ajuste").toFloat();
  saveParams();
  server.send(200, "text/plain", "Settings updated");
}
void handleNotFound() {
  server.sendHeader("Location", String("http://") + apIP.toString(), true);
  server.send(302, "text/plain", "");
}
void handleCalibrate() {
  float sum = 0;
  int N = 100;
  for (int i = 0; i < N; i++) {
    mpu.getAcceleration(&ax, &ay, &az);
    sum += atan2(ay, az) * 180.0 / PI;
    delay(5);
  }
  ANGLE_OFFSET = sum / N;
  mpu.getAcceleration(&ax, &ay, &az);
  ang_x = ang_x_prev = atan2(ay, az) * 180.0 / PI - ANGLE_OFFSET - ajuste_fino;
  integral = 0;
  derivative = 0;
  previous_error = 0;
  calibrateTime = millis();
  fallen = false;
  Serial.print("Nuevo OFFSET calculado: "); Serial.println(ANGLE_OFFSET, 2);
  server.send(200, "text/plain", "Offset calibrado. ¡Ya puedes encender los motores!");
}

void setupWiFiPortal() {
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(apIP, apIP, netMsk);
  WiFi.softAP(ssid, password);
  dnsServer.start(DNS_PORT, "*", apIP);
  server.on("/", handleRoot);
  server.on("/toggle", handleToggle);
  server.on("/status", handleStatus);
  server.on("/getsettings", handleGetSettings);
  server.on("/setsettings", handleSetSettings);
  server.on("/calibrate", handleCalibrate);
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.print("AP iniciado en: "); Serial.println(WiFi.softAPIP());
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  pinMode(pinSTBY, OUTPUT);
  pinMode(pinAIN1, OUTPUT);
  pinMode(pinAIN2, OUTPUT);
  pinMode(pinBIN1, OUTPUT);
  pinMode(pinBIN2, OUTPUT);
  pinMode(pinPWMA, OUTPUT);
  pinMode(pinPWMB, OUTPUT);

  digitalWrite(pinSTBY, HIGH);

  Wire.begin();
  mpu.initialize();

  // === OFFSET CALIBRADOS MANUALMENTE ===
  mpu.setXAccelOffset(-3554);
  mpu.setYAccelOffset(5858);
  mpu.setZAccelOffset(10188);
  mpu.setXGyroOffset(-30);
  mpu.setYGyroOffset(-234);
  mpu.setZGyroOffset(-30);

  delay(1000);

  float sum = 0;
  int N = 100;
  for (int i = 0; i < N; i++) {
    mpu.getAcceleration(&ax, &ay, &az);
    sum += atan2(ay, az) * 180.0 / PI;
    delay(5);
  }
  ANGLE_OFFSET = sum / N;
  Serial.print("OFFSET calculado: "); Serial.println(ANGLE_OFFSET, 2);

  mpu.getAcceleration(&ax, &ay, &az);
  ang_x = ang_x_prev = atan2(ay, az) * 180.0 / PI - ANGLE_OFFSET;
  calibrateTime = millis();
  lastControlTime = millis();

  EEPROM.begin(EEPROM_SIZE);
  loadParams();

  setupWiFiPortal();

  // --- Iniciar la tarea de control en FreeRTOS ---
  xTaskCreatePinnedToCore(
    ControlTask,
    "ControlTask",
    4096,
    NULL,
    2,
    NULL,
    1
  );
}

void loop() {
  dnsServer.processNextRequest();
  server.handleClient();
}

// ----- Tarea de control PID, filtro, autoparado, motores -----
void ControlTask(void * parameter) {
  unsigned long lastPrint = 0;
  lastControlTime = millis();
  while (true) {
    unsigned long now = millis();
    if (now - lastControlTime >= CONTROL_PERIOD_MS) {
      dt = (now - lastControlTime) / 1000.0;
      lastControlTime = now;

      // --- Sensor MPU6050 ---
      int16_t ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;
      mpu.getAcceleration(&ax_raw, &ay_raw, &az_raw);
      mpu.getRotation(&gx_raw, &gy_raw, &gz_raw);

      // --- Remapeo de ejes ---
      ax = -ay_raw;
      ay =  ax_raw;
      az =  az_raw;
      gx = -gy_raw;
      gy =  gx_raw;
      gz =  gz_raw;

      // --- Filtro complementario ---
      accel_ang_x = atan2(ay, az) * 180.0 / PI - ANGLE_OFFSET - ajuste_fino;

      if (now - calibrateTime < 2000) {
        ang_x = ang_x_prev = accel_ang_x;
      } else {
        float gyro_rate = gx / 131.0;
        ang_x = alpha * (ang_x_prev + gyro_rate * dt) + (1 - alpha) * accel_ang_x;
        ang_x_prev = ang_x;
      }

      // --- Autoparado ---
      if (abs(ang_x) > MAX_ANGLE) {
        if (fallStartTime == 0) fallStartTime = now;
        if (!fallen && now - fallStartTime > FALL_TIMEOUT) {
          fallen = true;
          motorsEnabled = false;
          Serial.println("Robot caído: motores deshabilitados.");
          integral = 0;
          derivative = 0;
          previous_error = 0;
        }
      } else {
        fallStartTime = 0;
        fallen = false;
      }

      // --- PID ---
      double error = setpoint - ang_x;
      integral += error * dt;
      if (integral > integral_max) integral = integral_max;
      if (integral < integral_min) integral = integral_min;
      derivative = (error - previous_error) / dt;
      output = Kp * error + Ki * integral + Kd * derivative;
      previous_error = error;

      // --- Motores ---
      if (motorsEnabled) {
        digitalWrite(pinSTBY, HIGH);
        driveMotors(-(int)output);
      } else {
        driveMotors(0);
        digitalWrite(pinSTBY, LOW);
      }

      // --- Frecuencia real de loop ---
      loopFrequency = 1.0 / dt;

      // --- Log serie cada 100 ms ---
      if (now - lastPrint > 100) {
        Serial.print("Ángulo: "); Serial.print(ang_x, 2);
        Serial.print(", PID: "); Serial.print(output, 2);
        Serial.print(", Offset: "); Serial.print(ANGLE_OFFSET, 2);
        Serial.print(", Setpoint: "); Serial.println(setpoint, 2);
        lastPrint = now;
      }
    }
    vTaskDelay(1); // Cede la CPU a otras tareas
  }
}
