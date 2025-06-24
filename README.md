Sistema de control PID para estabilizar un péndulo invertido, con interfaz web para ajuste remoto de parámetros. El proyecto incluye:

Control PID implementado en tiempo real

Interfaz web para ajuste de parámetros (Kp, Ki, Kd)

Calibración del ángulo cero

Protección contra caídas

Telemetría del estado del sistema

Hardware
Microcontrolador: ESP32

Sensor inercial: MPU6050 (acelerómetro + giroscopio)

Actuadores: 2 motores DC con puente H

Conexiones:

MPU6050: SDA (GPIO21), SCL (GPIO22)

Motores: PWMA (GPIO26), AIN1 (GPIO2), AIN2 (GPIO0), PWMB (GPIO25), BIN1 (GPIO16), BIN2 (GPIO17)

Configuración
Conectarse al AP "RobotAP" (contraseña: 12345678)

Acceder a http://192.168.4.1

Calibrar el ángulo cero antes de activar los motores

Parámetros ajustables
Kp, Ki, Kd: Constantes del controlador PID

Setpoint: Ángulo de referencia (en grados)

Ajuste fino: Offset adicional para calibración

Características de seguridad
Desactivación automática al detectar caída (>30° por más de 1s)

Anti-windup del término integral

Frecuencia de control fija a 100Hz

Estructura del código
ControlTask: Tarea FreeRTOS para el control PID

WebServer: Interfaz para ajuste remoto

Complementary Filter: Fusión de datos del MPU6050

Uso
Calibrar el sensor con el robot en posición vertical

Ajustar parámetros PID mediante la interfaz web

Activar los motores y monitorear el comportamiento
