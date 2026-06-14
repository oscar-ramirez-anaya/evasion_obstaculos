<h1 align="center">Actividad 4.2 — Evasion de Obstaculos</h1>
<h3 align="center">MR4010.10 Navegacion Autonoma</h3>

<br>

<table align="center">
  <tr>
    <td><b>Institucion</b></td>
    <td>Instituto Tecnologico y de Estudios Superiores de Monterrey</td>
  </tr>
  <tr>
    <td><b>Programa</b></td>
    <td>Maestria en Inteligencia Artificial</td>
  </tr>
  <tr>
    <td><b>Materia</b></td>
    <td>MR4010.10 — Navegacion Autonoma</td>
  </tr>
  <tr>
    <td><b>Profesor</b></td>
    <td>Dr. David Antonio-Torres</td>
  </tr>
  <tr>
    <td><b>Fecha</b></td>
    <td>Junio 2026</td>
  </tr>
</table>

<h3 align="center">Equipo</h3>

<table align="center">
  <tr><th>Nombre</th><th>Matricula</th></tr>
  <tr><td>Antonio Olvera Donlucas</td><td>A01795617</td></tr>
  <tr><td>Carlos Monir Radovich Saad</td><td>A01797569</td></tr>
  <tr><td>Andres Roberto Osuna Gonzalez</td><td>A01796264</td></tr>
  <tr><td>Oscar Alberto Ramirez Anaya</td><td>A01795438</td></tr>
</table>

---

## Indice

1. [Introduccion](#1-introduccion)
2. [Descripcion del codigo base](#2-descripcion-del-codigo-base)
3. [Modificaciones realizadas al codigo base](#3-modificaciones-realizadas-al-codigo-base)
4. [Arquitectura del sistema](#4-arquitectura-del-sistema)
5. [Reconocimiento visual del autobus](#5-reconocimiento-visual-del-autobus)
6. [LiDAR y deteccion de distancia](#6-lidar-y-deteccion-de-distancia)
7. [Evasion por seguimiento de pared derecha](#7-evasion-por-seguimiento-de-pared-derecha)
8. [Integracion del giroscopio](#8-integracion-del-giroscopio)
9. [Codigo del controlador autonomo](#9-codigo-del-controlador-autonomo)
10. [Resultados](#10-resultados)
11. [Video demostrativo](#11-video-demostrativo)
12. [Referencias](#12-referencias)
13. [Estructura del repositorio](#13-estructura-del-repositorio)

---

## 1. Introduccion

El objetivo de esta actividad es incorporar el **algoritmo de seguimiento de pared
(wall following)** como mecanismo de **evasion de obstaculos** en un vehiculo
autonomo simulado en Webots. Partiendo del controlador seguidor de linea con
control **PID** de la Actividad 2.1, se agregan los sensores y la logica necesarios
para que el vehiculo detecte un autobus estatico que bloquea el carril, lo rodee
por la derecha y, una vez superado, recupere su orientacion original y reanude el
seguimiento de linea.

El sistema integra cuatro subsistemas de percepcion y control:

1. **Camara con nodo Recognition** — identifica el autobus al frente del vehiculo.
2. **LiDAR (Sick LMS 291)** — mide la distancia al autobus.
3. **Giroscopio** — guarda y recupera la orientacion (angulo en el eje z).
4. **Sensores de distancia de un solo rayo** en el costado derecho — alimentan el
   algoritmo de seguimiento de pared.

La actividad replica los pasos solicitados en la rubrica: reconocer el autobus,
medir su distancia, disparar la evasion al cruzar un umbral, maniobrar por la
derecha y recuperar la trayectoria.

> **Declaracion de uso de inteligencia artificial.** Para el desarrollo de esta
> actividad se utilizo *Claude (Anthropic)* como herramienta de asistencia para la
> **generacion de codigo, depuracion y diseno de la maquina de estados**. Todo el
> codigo fue revisado y validado por el equipo, y la responsabilidad final del
> contenido recae en los autores.

---

## 2. Descripcion del codigo base

El controlador parte del seguidor de linea con PID desarrollado en la **Actividad 2.1**
(y reutilizado en la **Actividad 3.1**). De ese codigo base se conservan **sin cambios**
los siguientes elementos:

| Elemento | Funcion | Descripcion |
|----------|---------|-------------|
| Captura de camara | `get_image` | Convierte la imagen BGRA a matriz Numpy |
| Vision de carril | `procesar_lineas` | Grises -> Canny(50,150) -> ROI trapezoidal -> Hough |
| Calculo de error | `calcular_error_direccion` | Linea mas cercana al centro; filtra horizontales |
| Suavizado | EMA (alpha=0.6) | Evita volantazos por ruido en la deteccion |
| Control | PID inline | Kp=0.008, Ki=0.0, Kd=0.015, angulo limitado a ±0.5 rad |
| LiDAR | `process_lidar` | Sick LMS 291, zona central ±20 indices, < 20 m |
| Actuacion | `vehicle.Driver` | `setSteeringAngle` (rad) y `setCruisingSpeed` (km/h) |

El vehiculo es un **BmwX5** (modelo Ackermann) y la velocidad crucero por defecto
es de **30 km/h**.

---

## 3. Modificaciones realizadas al codigo base

| Aspecto | Actividad 2.1 / 3.1 (base) | Actividad 4.2 (esta entrega) |
|---------|----------------------------|------------------------------|
| Percepcion frontal | LiDAR + clasificacion SVM | LiDAR + **nodo Recognition** de la camara |
| Sensores laterales | Ninguno | **3 DistanceSensor de un rayo** en el costado derecho |
| Orientacion | No se usa el giroscopio | **Giroscopio** integrado para guardar/recuperar heading |
| Logica de control | Maquina de estados de frenado | **Maquina de estados de evasion** (5 estados) |
| Reaccion al obstaculo | Frenar / reducir velocidad | **Rodear** el obstaculo y recuperar la trayectoria |

En el mundo de Webots se agregaron **4 autobuses estaticos** (representados como
`Solid` con geometria `Box` y `recognitionColors`) y se incorporaron los **3 sensores
de distancia** al `sensorsSlotCenter` del BmwX5.

---

## 4. Arquitectura del sistema

El sistema combina tres pipelines que se ejecutan en cada paso de la simulacion.
El detalle completo (diagramas, tabla de dispositivos y parametros) esta en
[`docs/architecture.md`](docs/architecture.md).

```
Pipeline 1 — Seguimiento de carril (reutilizado)
  Camara -> Canny + ROI + Hough -> error -> EMA -> PID -> setSteeringAngle

Pipeline 2 — Deteccion del autobus
  LiDAR -> distancia        Camara+Recognition -> autobus al frente
  Disparo:  autobus reconocido AND lidar_dist < 15 m

Pipeline 3 — Evasion por pared derecha
  Giroscopio -> heading     Sensores derechos -> ley de pared -> setSteeringAngle
```

### Maquina de estados

```
LINE_FOLLOW ──(bus reconocido && lidar < 15 m)──────────────> APPROACH
APPROACH ────(lidar < 8 m)──────────────────────────────────> SAVE_HEADING
SAVE_HEADING ─(guarda heading, frena)───────────────────────> WALL_FOLLOW_RIGHT
WALL_FOLLOW_RIGHT ─(wall_found && rr >= 4 m && steps > 50)───> RECOVER_HEADING
RECOVER_HEADING ──(|heading - saved| < 0.08 && lidar libre)─> LINE_FOLLOW
```

---

## 5. Reconocimiento visual del autobus

La camara del vehiculo tiene habilitado el **nodo Recognition** de Webots. En el
controlador se activa con `camera.enableRecognition(timestep)` y en cada paso se
consultan los objetos detectados:

```python
def detect_bus_ahead(camera):
    objects = camera.getRecognitionObjects()
    cam_cx = camera.getWidth() / 2.0
    for obj in objects:
        if obj.getModel() == "autobus":          # campo 'model' del Solid
            pos = obj.getPositionOnImage()        # [pixel_x, pixel_y]
            colors = obj.getColors()              # [r, g, b, ...]
            if abs(pos[0] - cam_cx) < cam_cx * 0.5:  # tercio central
                return True, pos, colors
    return False, None, None
```

Cada autobus del mundo declara `model "autobus"` y un `recognitionColors` igual a su
color base, lo que permite que el nodo Recognition lo segmente y lo devuelva. Solo
se considera un autobus que aparece en el **tercio central** de la imagen, para
descartar autobuses perifericos que no bloquean el carril. La deteccion se imprime
en consola (evidencia del **paso 1** de la rubrica).

---

## 6. LiDAR y deteccion de distancia

La distancia al autobus se obtiene del **LiDAR Sick LMS 291** reutilizando la funcion
`process_lidar`, que promedia la zona central del barrido (±20 indices, < 20 m):

```python
range_data = lidar.getRangeImage()
# ... promedio de la zona central ...
avg_dist = obstacle_dist / collision_count
```

Cuando el autobus es reconocido **y** la distancia del LiDAR baja de **15 m**, el
vehiculo entra al estado `APPROACH`, reduce su velocidad a 10 km/h y reporta la
distancia en consola (evidencia del **paso 2** de la rubrica). Al bajar de **8 m**
se dispara la maniobra de evasion.

---

## 7. Evasion por seguimiento de pared derecha

Al cruzar el umbral de 8 m, el controlador deja de seguir la linea, **lee el
giroscopio para guardar la orientacion** y frena momentaneamente (estado
`SAVE_HEADING`, **paso 3** de la rubrica). Acto seguido entra a `WALL_FOLLOW_RIGHT`.

Se montaron **3 sensores de distancia de un solo rayo** en el costado derecho del
vehiculo, apuntando hacia afuera:

| Sensor | Posicion | Rol |
|--------|----------|-----|
| `ds_right_front` | frontal-derecha | detecta esquina; fuerza giro a la izquierda |
| `ds_right_mid` | centro-derecha | control de brecha (realimentacion principal) |
| `ds_right_rear` | trasero-derecha | **ultimo sensor**: indica el fin de la maniobra |

Cada sensor usa una `lookupTable [0 0 0, 5 5 0]` (rango lineal 0–5 m). La ley de
control de pared derecha tiene tres fases:

```python
if rm < WALL_CLEAR_DIST:
    wall_found = True                 # se engancha la cara izquierda del autobus

if not wall_found:
    wall_steering = -0.40             # Fase 1: buscar girando a la izquierda
elif rf < CORNER_THRESHOLD:
    wall_steering = -MAX_ANGLE        # Fase 2: esquina cercana -> giro fuerte izquierda
else:
    gap_error = rm - TARGET_WALL_DIST # Fase 3: mantener ~1.5 m de brecha
    wall_steering = K_WALL * gap_error
    wall_steering = max(-MAX_ANGLE, min(MAX_ANGLE, wall_steering))
```

El algoritmo **termina** cuando el autobus ya fue enganchado, el **ultimo sensor**
(trasero-derecho) ya no ve obstaculo (`rr >= 4 m`) y transcurrieron suficientes
pasos (evidencia del **paso 4** de la rubrica). Entonces se pasa a `RECOVER_HEADING`.

---

## 8. Integracion del giroscopio

El **giroscopio** del slot central entrega la velocidad angular en el eje z. El
controlador la integra en cada paso para estimar el `heading` (orientacion acumulada):

```python
# getValues() -> [wx, wy, wz] en rad/s (marco local del vehiculo)
heading += gyro.getValues()[2] * (timestep / 1000.0)
```

Al iniciar la evasion se guarda `saved_heading = heading`. En el estado
`RECOVER_HEADING` el vehiculo gira proporcionalmente al error de orientacion hasta
re-alinearse con la direccion previa a la evasion:

```python
heading_error = heading - saved_heading
recover_steering = K_RECOVER * heading_error
recover_steering = max(-MAX_ANGLE, min(MAX_ANGLE, recover_steering))
```

Cuando `|heading_error| < 0.08 rad` y el LiDAR esta libre, se reinicia el integrador
del PID y se reanuda el seguimiento de carril (evidencia del **paso 5** de la rubrica).

---

## 9. Codigo del controlador autonomo

El controlador completo se encuentra en
[`controllers/evasion_obstaculos/evasion_obstaculos.py`](controllers/evasion_obstaculos/evasion_obstaculos.py).
Esta organizado en cuatro secciones: (1) constantes, (2) seguimiento de lineas
reutilizado, (3) LiDAR y reconocimiento, y (4) bucle principal con la maquina de
estados de evasion. Cada bloque incluye comentarios que explican los argumentos de
las funciones y los parametros modificados.

### Controles y salida en consola

| Tecla | Accion |
|-------|--------|
| ↑ | Aumentar velocidad crucero (+5 km/h) |
| ↓ | Disminuir velocidad crucero (−5 km/h) |

Ejemplo de salida en consola durante una evasion completa:

```
[RECOGNITION] Autobus detectado | pos_imagen=(128,70) | color=(1.00,0.50,0.00)
[APPROACH] Autobus al frente | LiDAR=12.1m
[SAVE_HEADING] Orientacion guardada = 0.0021 rad — iniciando evasion
[WALL_FOLLOW_RIGHT] LiDAR=--- heading=0.341 saved=0.002 | rf=5.00 rm=1.48 rr=5.00
[WALL_FOLLOW] Obstaculo superado — iniciando recuperacion de orientacion
[RECOVER] Orientacion recuperada — reanudando seguimiento de carril
```

---

## 10. Resultados

El vehiculo completa el ciclo de evasion de forma autonoma:

1. **Reconoce** cada autobus mediante el nodo Recognition y lo reporta en consola.
2. **Mide** la distancia con el LiDAR y reduce la velocidad al acercarse.
3. **Guarda** la orientacion con el giroscopio al cruzar el umbral de 8 m.
4. **Rodea** el autobus por la derecha manteniendo ~1.5 m de brecha con los sensores
   laterales, hasta que el sensor trasero queda libre.
5. **Recupera** la orientacion previa y reanuda el seguimiento de carril.

Los umbrales y las posiciones de los autobuses son valores iniciales razonables que
pueden ajustarse visualmente en Webots segun el trazado especifico de la ruta.

---

## 11. Video demostrativo

[Enlace al video en YouTube](https://youtu.be/PENDIENTE)

El video (< 5 min) explica cada uno de los pasos del algoritmo de evasion y demuestra
que el vehiculo conserva el comportamiento de seguidor de linea antes y despues de
la maniobra.

---

## 12. Referencias

- Cyberbotics Ltd. *Webots User Guide* (R2025a). https://cyberbotics.com/doc/guide/index
- Cyberbotics Ltd. *Webots Reference Manual — Camera Recognition*. https://cyberbotics.com/doc/reference/recognition
- Cyberbotics Ltd. *Webots Reference Manual — DistanceSensor*. https://cyberbotics.com/doc/reference/distancesensor
- Cyberbotics Ltd. *Webots Reference Manual — Gyro*. https://cyberbotics.com/doc/reference/gyro
- SICK AG. *LMS291 Laser Measurement System — Technical Description*.
- Astrom, K. J., & Murray, R. M. (2008). *Feedback Systems: An Introduction for Scientists and Engineers*. Princeton University Press.

---

## 13. Estructura del repositorio

```
evasion_obstaculos/
├── README.md                          # Este reporte
├── LICENSE                            # Apache 2.0
├── .gitignore
├── docs/
│   └── architecture.md                # Diagramas, tabla de sensores y parametros
├── controllers/
│   └── evasion_obstaculos/
│       ├── evasion_obstaculos.py      # Controlador Webots (extern)
│       ├── requirements.txt           # numpy, opencv-python
│       └── runtime.ini                # PYTHONPATH local (ignorado por git)
└── worlds/
    └── city_2025b_evasion.wbt         # Mundo con 4 autobuses + sensores derechos
```

### Ejecucion

1. Crear `controllers/evasion_obstaculos/runtime.ini` con el `PYTHONPATH` al entorno
   de Python 3.13 que tenga `numpy` y `opencv-python` instalados:

   ```ini
   [environment variables]
   PYTHONPATH=/ruta/a/tu/env/lib/python3.13/site-packages
   ```

2. Abrir `worlds/city_2025b_evasion.wbt` en **Webots R2025a**. El controlador
   `<extern>` del BmwX5 se adjunta automaticamente.
3. Iniciar la simulacion y observar la consola para validar los cinco pasos del
   algoritmo de evasion.

<br>
<p align="center"><i>Instituto Tecnologico y de Estudios Superiores de Monterrey
— Maestria en Inteligencia Artificial — Junio 2026</i></p>
