# AGENTS.md

## Contexto del proyecto

Este repositorio contiene el firmware de un robot balancín de dos ruedas basado en ESP32.

Hardware principal:

- ESP32
- MPU6500
- Dos motorreductores DC de 500 rpm con encoders
- Puente H doble con control por PWM
- Batería
- Interfaz web para depuración y ajuste de parámetros

El proyecto se desarrollará en PlatformIO usando Arduino Framework.

El firmware debe usar FreeRTOS para separar claramente las tareas principales del sistema:

- Core 0: manejo web, servidor, interfaz de depuración y recepción de comandos.
- Core 1: control del robot, lectura del MPU6500, lectura de encoders, seguridad, motores y PID.

---

## Objetivo del agente

Ayudar a construir el firmware paso a paso, de forma didáctica, modular y fácil de depurar.

El código debe priorizar:

1. Claridad.
2. Explicación sencilla.
3. Separación por módulos.
4. Pruebas por etapas.
5. Seguridad antes de activar motores.
6. Uso adecuado de FreeRTOS.
7. Facilidad para modificar parámetros desde una interfaz web.

No se debe entregar código excesivamente complejo si no es necesario.

---

## Estilo general de programación

- Usar C++ con Arduino Framework en PlatformIO.
- Evitar clases propias al inicio del proyecto.
- Separar el proyecto en librerías locales usando archivos `.h` y `.cpp`.
- Usar funciones claras y nombres descriptivos.
- Usar `namespace` solo si ayuda a organizar el código.
- Mantener `src/main.cpp` limpio y corto.
- No mezclar lógica de sensores, motores, PID e interfaz web en el mismo archivo.
- Comentar el código explicando el propósito de cada bloque importante.
- Evitar retardos bloqueantes con `delay()`.
- Usar `vTaskDelay()` dentro de tareas FreeRTOS.
- Usar `millis()` o `micros()` para temporización fina si es necesario.
- No activar motores automáticamente al iniciar el ESP32.
- Toda prueba de motores debe requerir una acción explícita desde el código o la interfaz web.

---

## Librerías externas recomendadas

Agregar en `platformio.ini`:

```ini
lib_deps =
    br3ttb/PID
    madhephaestus/ESP32Encoder
    bolderflight/Bolder Flight Systems Invensense IMU
```

Uso esperado:

- `PID`: control PID de equilibrio.
- `ESP32Encoder`: lectura de encoders.
- `Bolder Flight Systems Invensense IMU`: lectura del MPU6500.
- `Wire.h`: comunicación I2C.
- `WiFi.h`: conexión WiFi.
- `WebServer.h` o alternativa similar: interfaz web de depuración.
- FreeRTOS incluido en ESP32 Arduino Framework.

---

## Arquitectura FreeRTOS obligatoria

El proyecto debe usar dos tareas principales:

```text
Core 0:
  Tarea web/debug
  - Servidor web
  - Interfaz de depuración
  - Recepción de comandos
  - Modificación de parámetros
  - Visualización de variables

Core 1:
  Tarea de control del robot
  - Lectura del MPU6500
  - Lectura de encoders
  - Filtro complementario
  - Seguridad
  - PID
  - Control de motores
```

Reglas:

- La tarea web no debe controlar directamente los motores.
- La tarea web solo debe modificar comandos o parámetros compartidos.
- La tarea de control es la única autorizada para escribir salidas a los motores.
- Las variables compartidas entre tareas deben protegerse con mutex.
- El control del robot debe tener frecuencia estable.
- El servidor web no debe bloquear el control del robot.

Tareas sugeridas:

```cpp
void webTask(void* parameter);
void controlTask(void* parameter);
```

Creación sugerida:

```cpp
xTaskCreatePinnedToCore(
    webTask,
    "WebTask",
    8192,
    NULL,
    1,
    NULL,
    0
);

xTaskCreatePinnedToCore(
    controlTask,
    "ControlTask",
    8192,
    NULL,
    2,
    NULL,
    1
);
```

---

## Datos compartidos entre tareas

Crear una estructura para estado del robot:

```cpp
struct RobotState {
    float ax;
    float ay;
    float az;

    float gx;
    float gy;
    float gz;

    float angle;
    float gyroRate;

    long leftEncoder;
    long rightEncoder;

    float leftSpeed;
    float rightSpeed;

    int leftPwm;
    int rightPwm;

    bool imuReady;
    bool motorsEnabled;
    bool safetyFault;

    char faultMessage[80];
};
```

Crear una estructura para comandos y parámetros:

```cpp
struct RobotCommand {
    bool enableMotors;
    bool stopMotors;
    bool resetEncoders;
    bool calibrateImu;

    float requestedForward;
    float requestedTurn;

    double kp;
    double ki;
    double kd;

    double angleSetpoint;
    int maxPwm;
};
```

Reglas:

- `RobotState` lo actualiza principalmente la tarea de control.
- `RobotCommand` lo actualiza principalmente la tarea web.
- Toda lectura/escritura compartida debe hacerse con mutex.
- No acceder a estas estructuras desde dos tareas sin protección.

Mutex sugerido:

```cpp
SemaphoreHandle_t robotDataMutex;
```

Funciones sugeridas:

```cpp
void sharedData_begin();
void sharedData_setState(const RobotState& state);
RobotState sharedData_getState();
void sharedData_setCommand(const RobotCommand& command);
RobotCommand sharedData_getCommand();
void sharedData_requestStop();
void sharedData_clearStopRequest();
```

---

## Estructura del proyecto

Usar esta estructura base:

```text
robot-balancin/
├── AGENTS.md
├── platformio.ini
├── include/
│   └── config.h
├── src/
│   └── main.cpp
└── lib/
    ├── shared_data/
    │   ├── shared_data.h
    │   └── shared_data.cpp
    ├── imu6500/
    │   ├── imu6500.h
    │   └── imu6500.cpp
    ├── motors/
    │   ├── motors.h
    │   └── motors.cpp
    ├── encoders/
    │   ├── encoders.h
    │   └── encoders.cpp
    ├── balance_pid/
    │   ├── balance_pid.h
    │   └── balance_pid.cpp
    ├── drive_control/
    │   ├── drive_control.h
    │   └── drive_control.cpp
    ├── web_debug/
    │   ├── web_debug.h
    │   └── web_debug.cpp
    └── safety/
        ├── safety.h
        └── safety.cpp
```

---

## Responsabilidad de cada módulo

### `include/config.h`

Contiene configuración global del proyecto:

- Pines del MPU6500.
- Pines del puente H.
- Pines de encoders.
- Frecuencia PWM.
- Resolución PWM.
- Límites de PWM.
- Parámetros iniciales del PID.
- Ángulo máximo seguro.
- Tiempo de muestreo del control.
- Parámetros WiFi.
- Constantes de calibración.
- Inversión de motores.
- Inversión de encoders.
- Configuración de tareas FreeRTOS.

No colocar lógica de control dentro de este archivo.

Ejemplos de constantes:

```cpp
#define CONTROL_TASK_CORE 1
#define WEB_TASK_CORE 0

#define CONTROL_PERIOD_MS 10

#define MAX_SAFE_ANGLE 35.0
#define INITIAL_MAX_PWM 120

#define INVERT_LEFT_MOTOR false
#define INVERT_RIGHT_MOTOR false

#define INVERT_LEFT_ENCODER false
#define INVERT_RIGHT_ENCODER true
```

---

### `lib/shared_data`

Responsable de:

- Crear el mutex de datos compartidos.
- Guardar el estado actual del robot.
- Guardar comandos recibidos desde la web.
- Permitir lectura segura desde ambas tareas.
- Permitir escritura segura desde ambas tareas.

Funciones sugeridas:

```cpp
void sharedData_begin();

void sharedData_setState(const RobotState& state);
RobotState sharedData_getState();

void sharedData_setCommand(const RobotCommand& command);
RobotCommand sharedData_getCommand();

void sharedData_updateMotorsEnabled(bool enabled);
void sharedData_requestStop();
void sharedData_clearStopRequest();
```

Reglas:

- No compartir variables globales sin mutex.
- No mantener el mutex bloqueado durante operaciones lentas.
- Copiar datos rápidamente y liberar el mutex.

---

### `lib/imu6500`

Responsable de:

- Inicializar el MPU6500.
- Calibrar offset del giroscopio.
- Leer acelerómetro y giroscopio.
- Calcular ángulo por acelerómetro.
- Calcular ángulo con filtro complementario.
- Entregar el ángulo actual del robot.
- Reportar si el sensor está funcionando.

Funciones sugeridas:

```cpp
bool imu_begin();
void imu_calibrateGyro();
void imu_update();
float imu_getAngle();
float imu_getGyroRate();
bool imu_isReady();

float imu_getAx();
float imu_getAy();
float imu_getAz();

float imu_getGx();
float imu_getGy();
float imu_getGz();
```

Reglas:

- El MPU debe calibrarse con el robot quieto.
- La calibración no debe ejecutarse automáticamente si los motores están activos.
- El filtro complementario debe usar un `dt` estable.
- El eje usado para el ángulo debe estar claramente documentado.

---

### `lib/motors`

Responsable de:

- Configurar pines del puente H.
- Configurar PWM del ESP32.
- Mover motor izquierdo.
- Mover motor derecho.
- Aplicar dirección según signo del PWM.
- Limitar PWM máximo.
- Frenar motores.
- Apagar motores.

Funciones sugeridas:

```cpp
void motors_begin();

void motors_setLeft(int pwm);
void motors_setRight(int pwm);
void motors_setBoth(int leftPwm, int rightPwm);

void motors_stop();
void motors_disable();

int motors_getLeftPwm();
int motors_getRightPwm();
```

Reglas:

- PWM positivo: un sentido de giro.
- PWM negativo: sentido contrario.
- PWM cero: motor detenido.
- Si los motores giran al revés, corregir desde constantes de inversión en `config.h`.
- La tarea web no debe llamar directamente a `motors_setLeft()` ni `motors_setRight()`.
- Solo la tarea de control debe escribir en los motores.

---

### `lib/encoders`

Responsable de:

- Inicializar encoders.
- Leer conteos crudos.
- Leer conteos corregidos.
- Reiniciar conteos.
- Calcular velocidad de cada rueda.
- Calcular velocidad promedio.
- Verificar sentido de giro.

Funciones sugeridas:

```cpp
void encoders_begin();
void encoders_update();

long encoders_getRawLeftCount();
long encoders_getRawRightCount();

long encoders_getLeftCount();
long encoders_getRightCount();

float encoders_getLeftSpeed();
float encoders_getRightSpeed();
float encoders_getAverageSpeed();

void encoders_reset();
```

Reglas:

- El avance del robot debe producir conteos positivos en ambos encoders corregidos.
- Si un encoder cuenta al revés, corregir desde `config.h`.
- No corregir el signo directamente dentro de la lógica principal.

---

### `lib/balance_pid`

Responsable de:

- Configurar la librería PID.
- Mantener variables `setpoint`, `input` y `output`.
- Calcular salida de equilibrio.
- Permitir modificar `Kp`, `Ki`, `Kd`.
- Permitir modificar el setpoint de ángulo.
- Limitar salida máxima.

Funciones sugeridas:

```cpp
void balancePid_begin();

void balancePid_update(float angle);
int balancePid_getOutput();

void balancePid_setTunings(double kp, double ki, double kd);
void balancePid_setSetpoint(double setpoint);
void balancePid_setOutputLimits(int minOutput, int maxOutput);

double balancePid_getKp();
double balancePid_getKi();
double balancePid_getKd();
double balancePid_getSetpoint();
```

Reglas:

- La salida del PID debe poder ser negativa y positiva.
- No usar salida solo de `0` a `255`.
- Empezar con límites suaves, por ejemplo `-120` a `120`.
- El PID de equilibrio debe probarse antes de agregar conducción.

---

### `lib/drive_control`

Responsable de:

- Convertir comandos de conducción en setpoint de ángulo y giro.
- Gestionar avance, retroceso y giro.
- Combinar salida PID con comando de giro.
- Entregar PWM final para motor izquierdo y derecho.

Funciones sugeridas:

```cpp
void drive_setCommand(float forward, float turn);
void drive_update(int balanceOutput);

int drive_getLeftPwm();
int drive_getRightPwm();

float drive_getAngleOffset();
int drive_getTurnPwm();

void drive_stopCommand();
```

Reglas:

- Avance/retroceso modifica ligeramente el ángulo objetivo.
- Giro modifica la diferencia entre motores.
- No implementar conducción hasta que el equilibrio básico funcione.
- Los comandos deben suavizarse para evitar cambios bruscos.

---

### `lib/web_debug`

Responsable de:

- Crear una interfaz web local.
- Mostrar variables del robot.
- Modificar parámetros.
- Solicitar habilitar/deshabilitar motores.
- Solicitar calibración.
- Cambiar parámetros PID.
- Cambiar límites de PWM.
- Ver estado de seguridad.

Debe incluir como mínimo:

- Ángulo actual.
- Aceleración `ax`, `ay`, `az`.
- Giroscopio corregido `gx`, `gy`, `gz`.
- Velocidad del giroscopio usada para balance.
- Conteo de encoder izquierdo.
- Conteo de encoder derecho.
- Velocidad de rueda izquierda.
- Velocidad de rueda derecha.
- PWM izquierdo.
- PWM derecho.
- Estado de motores.
- Estado del MPU.
- Estado de seguridad.
- Parámetros `Kp`, `Ki`, `Kd`.
- Setpoint de equilibrio.
- Límite máximo de PWM.
- Botón de calibración del MPU.
- Botón de detener motores.
- Botón de habilitar motores.
- Botón de resetear encoders.
- Botones de prueba de motor izquierdo y derecho.

Funciones sugeridas:

```cpp
void webDebug_begin();
void webDebug_handleClient();
```

Reglas:

- Este módulo se ejecuta en la tarea web del Core 0.
- No debe controlar motores directamente.
- Debe escribir comandos en `shared_data`.
- Debe leer el estado del robot desde `shared_data`.
- No debe bloquear la tarea de control.

---

### `lib/safety`

Responsable de proteger el robot y al usuario.

Debe apagar motores si:

- El ángulo supera el límite seguro.
- El MPU no responde.
- El robot está acostado.
- La interfaz ordena parada.
- Hay pérdida de control o timeout.
- El sistema aún no está calibrado.

Funciones sugeridas:

```cpp
void safety_begin();

void safety_update(float angle, bool imuReady, const RobotCommand& command);

bool safety_canRunMotors();
bool safety_isFaultActive();

const char* safety_getFaultMessage();

void safety_clearFault();
```

Reglas:

- La seguridad tiene prioridad sobre el PID.
- Si `safety_canRunMotors()` es falso, los motores deben apagarse.
- No permitir que el PID active motores si hay falla activa.
- La parada desde la web debe tener efecto inmediato en la tarea de control.

---

## Fases de desarrollo

### Fase 1: pruebas básicas de hardware

Crear pruebas independientes para:

- Lectura del MPU6500.
- Movimiento de motor izquierdo.
- Movimiento de motor derecho.
- Cambio de sentido de giro.
- PWM progresivo.
- Lectura de encoder izquierdo.
- Lectura de encoder derecho.
- Verificación de tierra común y alimentación.

Resultado esperado:

- Cada componente funciona por separado.

---

### Fase 2: estructura base, FreeRTOS e interfaz web

Crear la estructura modular del proyecto.

Implementar:

- Tarea web en Core 0.
- Tarea de control en Core 1.
- Estructura de datos compartidos.
- Mutex para proteger datos.
- Interfaz web básica.

La interfaz web debe permitir:

- Ver variables en tiempo real.
- Habilitar/deshabilitar motores.
- Solicitar pruebas simples.
- Resetear encoders.
- Calibrar MPU.
- Detener todo con un botón.

Resultado esperado:

- El robot puede depurarse sin recompilar constantemente.
- El manejo web no bloquea el control del robot.

---

### Fase 3: MPU6500 para el robot balancín

Implementar:

- Inicialización del MPU6500.
- Calibración del giroscopio.
- Cálculo de ángulo por acelerómetro.
- Filtro complementario.
- Offset de ángulo vertical.
- Visualización del ángulo en la web.

Resultado esperado:

- El ángulo del robot es estable y útil para control.

---

### Fase 4: motores y encoders

Implementar:

- Control PWM de ambos motores.
- Corrección de sentido de giro.
- Lectura de encoders.
- Cálculo de velocidad de ruedas.
- Visualización en web.
- Pruebas de avance, retroceso y giro sin balance.

Resultado esperado:

- Los motores responden correctamente y los encoders miden el movimiento.

---

### Fase 5: PID de equilibrio

Implementar:

- PID usando la librería `PID`.
- Entrada: ángulo actual.
- Setpoint: ángulo vertical calibrado.
- Salida: PWM positivo o negativo.
- Ajuste de `Kp`, `Ki`, `Kd` desde la web.
- Límites de salida configurables.
- Parada segura si el ángulo es excesivo.

Resultado esperado:

- El robot intenta mantenerse en equilibrio sin comandos de conducción.

---

### Fase 6: conducción

Implementar:

- Avance.
- Retroceso.
- Giro a la izquierda.
- Giro a la derecha.
- Control desde interfaz web.
- Cambio suave del setpoint de ángulo.
- Diferencia de PWM entre motores para girar.

Resultado esperado:

- El robot puede mantenerse de pie y desplazarse de forma controlada.

---

### Fase 7: ajuste, registro y documentación

Implementar:

- Registro básico de valores importantes por Serial.
- Documentación de pines.
- Documentación de calibración.
- Documentación de ajuste PID.
- Tabla de pruebas realizadas.
- Valores recomendados iniciales.

Resultado esperado:

- Proyecto entendible, reproducible y fácil de presentar.

---

## Reglas de seguridad del firmware

- Los motores deben iniciar deshabilitados.
- Debe existir una función global de parada.
- Debe existir un botón de parada en la interfaz web.
- No activar motores si el MPU falla.
- No activar motores si no se ha calibrado el ángulo.
- Apagar motores si el ángulo supera el límite seguro.
- Limitar PWM máximo durante pruebas iniciales.
- Evitar cambios bruscos de setpoint.
- No usar `delay()` dentro del loop de control.
- La tarea web nunca debe tener prioridad sobre la tarea de control.
- El control del robot debe seguir funcionando aunque la web tarde en responder.

---

## Reglas para Codex

Cuando se le pida implementar una fase:

1. Revisar primero esta guía.
2. Crear o modificar solo los archivos necesarios.
3. Mantener el proyecto modular.
4. No reescribir todo el proyecto si solo se pidió una función.
5. Explicar brevemente qué archivos cambió.
6. Incluir instrucciones claras para probar.
7. No avanzar a la siguiente fase sin que la fase actual sea verificable.
8. Priorizar código simple y entendible.
9. No ocultar lógica importante en funciones difíciles de entender.
10. Mantener nombres consistentes entre módulos.
11. Respetar la arquitectura FreeRTOS.
12. No controlar motores desde la tarea web.
13. No acceder a datos compartidos sin mutex.
14. No implementar PID antes de que el ángulo sea estable.
15. No implementar conducción antes de que el equilibrio funcione.

---

## Formato esperado de respuesta de Codex

Cuando Codex entregue cambios, debe responder así:

```text
Cambios realizados:
- ...

Archivos modificados:
- ...

Cómo probar:
1. ...
2. ...
3. ...

Qué deberías observar:
- ...

Siguiente paso recomendado:
- ...
```

---

## Notas importantes

- El MPU6500 no tiene magnetómetro, pero eso no es un problema para un robot balancín.
- Para equilibrio se usa principalmente acelerómetro + giroscopio.
- Los encoders se usan para velocidad, desplazamiento y conducción.
- Primero debe funcionar el equilibrio quieto.
- Después se agrega conducción.
- El PID debe ajustarse con el robot en un soporte o con las ruedas libres al inicio.
- La interfaz web es una herramienta de depuración, no debe reemplazar las protecciones del firmware.
- FreeRTOS se usa para evitar que la interfaz web bloquee el control del robot.
