# Péndulo invertido: control PID e interfaz web

Sistema de control PID para estabilizar un péndulo invertido con un ESP32. Incluye una interfaz web para ajustar los parámetros, telemetría y rutinas de calibración.

## Características

- Control PID a 100 Hz.
- Ajuste remoto de Kp, Ki, Kd, setpoint y offset.
- Calibración del ángulo cero.
- Desactivación automática cuando detecta una caída.
- Telemetría del estado del sistema.
- Anti-windup para limitar la acumulación del término integral.

## Hardware

- **Microcontrolador:** ESP32
- **Sensor inercial:** MPU6050 (acelerómetro y giroscopio)
- **Actuadores:** dos motores DC con puente H

### Conexiones GPIO

#### MPU6050 (I2C)

| Señal | GPIO |
|------:|:----:|
| SDA   | 21   |
| SCL   | 22   |

#### Motores (puente H)

| Motor | Señal | GPIO |
|------:|:-----:|
| A     | PWMA  | 26   |
| A     | AIN1  | 2    |
| A     | AIN2  | 0    |
| B     | PWMB  | 25   |
| B     | BIN1  | 16   |
| B     | BIN2  | 17   |

## Puesta en marcha

1. Conéctate a la red WiFi **RobotAP** (contraseña: 12345678).
2. Abre http://192.168.4.1 en el navegador.
3. Con el robot en posición vertical, calibra el ángulo cero antes de activar los motores.
4. Ajusta los parámetros PID desde la interfaz y activa el control.

![Péndulo invertido en funcionamiento](https://github.com/user-attachments/assets/b4e0e8b0-0213-478d-b117-d779a0eff4ad)

## Parámetros ajustables

| Parámetro | Descripción |
|----------:|-------------|
| **Kp, Ki, Kd** | Constantes del controlador PID |
| **Setpoint** | Ángulo objetivo, en grados |
| **Offset** | Ajuste fino para la calibración |

## Protección y arquitectura

La protección desactiva el control cuando el ángulo supera los 30° durante más de un segundo. El control se ejecuta en una tarea de FreeRTOS a 100 Hz; el servidor web permite consultar el estado y cambiar los parámetros; y un filtro complementario combina el acelerómetro y el giroscopio para estimar el ángulo.

## Uso recomendado

Coloca el robot en vertical, calibra el sensor y empieza con valores PID conservadores. Activa los motores mientras observas la telemetría y ajusta el setpoint y el offset de forma gradual.

Si entra en modo de protección, revisa la calibración, el sentido de giro, la saturación del control y las vibraciones o el ruido del MPU6050.

![Detalle del péndulo invertido](https://github.com/user-attachments/assets/a7679310-da4c-473d-b857-04c6abeb0e5f)
