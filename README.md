# 🤖 Péndulo Invertido — Control PID + Interfaz Web (ESP32)

Sistema de control **PID en tiempo real** para estabilizar un **péndulo invertido**, con **interfaz web** para ajustar parámetros remotamente. Incluye telemetría, calibración y medidas de seguridad para evitar daños por caídas.

---

## ✨ Características principales

- ✅ Control **PID** implementado en tiempo real (**100 Hz**)
- 🌐 Interfaz web para ajuste remoto de **Kp, Ki, Kd**, setpoint y offset
- 🎯 **Calibración del ángulo cero**
- 🛡️ Protección contra caídas (auto-desactivación)
- 📡 **Telemetría** del estado del sistema
- 🧯 Anti-windup del término integral (protección del PID)

---

## 🧩 Hardware

- **Microcontrolador:** ESP32  
- **Sensor inercial:** MPU6050 (acelerómetro + giroscopio)  
- **Actuadores:** 2× motores DC con puente H  

### 🔌 Conexiones (GPIO)

#### MPU6050 (I2C)
| Señal | GPIO |
|------:|:----:|
| SDA   | 21   |
| SCL   | 22   |

#### Motores (Puente H)
| Motor | Señal | GPIO |
|------:|:-----:|:----:|
| A     | PWMA  | 26   |
| A     | AIN1  | 2    |
| A     | AIN2  | 0    |
| B     | PWMB  | 25   |
| B     | BIN1  | 16   |
| B     | BIN2  | 17   |

---

## ⚙️ Puesta en marcha (Web UI)

1. Conéctate al WiFi **RobotAP**  
   - **Contraseña:** `12345678`
2. Abre en el navegador:  
   - `http://192.168.4.1`
3. **Calibra el ángulo cero** con el robot **vertical** antes de activar motores.
4. Ajusta parámetros PID y activa el control.
   ![WhatsApp Image 2025-12-02 at 19 46 14](https://github.com/user-attachments/assets/b4e0e8b0-0213-478d-b117-d779a0eff4ad)


---

## 🎛️ Parámetros ajustables

| Parámetro | Descripción |
|----------:|-------------|
| **Kp, Ki, Kd** | Constantes del controlador PID |
| **Setpoint** | Ángulo objetivo (en grados) |
| **Ajuste fino (Offset)** | Offset adicional para calibración fina |

---

## 🛡️ Seguridad

- 🧱 **Desactivación automática** si detecta caída: **> 30° durante más de 1 s**
- 🧯 **Anti-windup** del integrador para evitar saturación
- ⏱️ Frecuencia de control fija: **100 Hz**

---

## 🏗️ Arquitectura del software

- **ControlTask** → Tarea FreeRTOS que ejecuta el control PID a 100 Hz  
- **WebServer** → Interfaz web para lectura/ajuste de parámetros y estado  
- **Complementary Filter** → Fusión MPU6050 (acelerómetro + giroscopio) para estimar el ángulo  

---

## 🚀 Uso recomendado

1. Coloca el robot **en vertical** y realiza la **calibración**.
2. Ajusta **Kp, Ki, Kd** desde la web (empieza con valores conservadores).
3. Activa motores y **monitoriza la telemetría**.
4. Afina setpoint/offset hasta lograr estabilidad.

---

## 📌 Notas

- Si el sistema entra en modo protección, revisa:
  - calibración del cero
  - dirección de los motores
  - saturación del control (Kp/Ki demasiado altos)
  - vibraciones mecánicas/ruido del MPU6050

---

## 📷



![WhatsApp Image 2025-12-02 at 19 46 15](https://github.com/user-attachments/assets/a7679310-da4c-473d-b857-04c6abeb0e5f)


