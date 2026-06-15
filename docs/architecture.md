# Arquitectura — Actividad 4.2 Evasión de Obstáculos

Documento técnico del controlador autónomo en Webots. Describe los tres pipelines
del sistema, la máquina de estados de evasión y la configuración de sensores.

## Vehículo y mundo

- **Vehículo:** `BmwX5` (modelo Ackermann, API `vehicle.Driver`).
- **Mundo:** `worlds/city_2025b_evasion.wbt` (Webots R2025a, EXTERNPROTO R2023b),
  forkeado de la Actividad 3.1 agregando 4 autobuses estáticos y 3 sensores de
  distancia de un solo rayo en el costado derecho.
- **Controlador:** `<extern>` (proceso Python externo), adjuntado vía `runtime.ini`.

## Pipeline 1 — Seguimiento de carril (reutilizado de la Act. 2.1)

```
Cámara(256x128, BGRA)
   -> get_image
   -> procesar_lineas:  Grises -> GaussianBlur -> Canny(50,150)
                        -> ROI trapezoidal -> HoughLinesP
   -> calcular_error_direccion  (línea más cercana al centro, filtra horizontales)
   -> EMA (alpha=0.6)
   -> PID (Kp=0.008, Ki=0.0, Kd=0.015)  -> setSteeringAngle
```

## Pipeline 2 — Detección del autobús (LiDAR + Recognition)

```
LiDAR Sick LMS 291
   -> process_lidar (zona central ±20 índices, < 20 m)  -> lidar_dist

Cámara + nodo Recognition
   -> camera.enableRecognition
   -> camera.getRecognitionObjects
   -> detect_bus_ahead (model == "autobus", tercio central de la imagen)
   -> (bus_recognized, pos_imagen, colores)

Disparo de evasión:  bus_recognized  AND  lidar_dist < APPROACH_DIST (15 m)
```

## Pipeline 3 — Evasión acotada por rumbo (giroscopio)

```
Giroscopio (eje z)
   -> heading += gyro.getValues()[2] * dt        (yaw integrado)

Sensores de distancia de un rayo
   -> ds_right_front / ds_right_mid / ds_right_rear  (costado derecho, 0-5 m)
   -> ds_left                                        (barandal izquierdo, 0-8 m)
   -> evasion acotada por rumbo  -> setSteeringAngle / setCruisingSpeed(EVADE_SPEED)
```

## Máquina de estados (2 estados)

```
LINE_FOLLOW ──(bus reconocido && lidar < 16 m)──────────────> EVADE  (guarda heading)
EVADE ──(rebasado: costado derecho libre && rumbo recto)────> LINE_FOLLOW
```

La evasión usa dos sub-fases internas y un freno de seguridad:

| Sub-fase / regla | Condición | Acción |
|------------------|-----------|--------|
| Freno de seguridad | `lidar < 4.5 m` y aún sin desviarse | frena (0.6) + giro `-MAX_ANGLE` para salir sin chocar |
| OUT (salir) | `heading_dev < DEV_TARGET` y `ds_left ≥ 2 m` | girar izquierda `-SEARCH_STEER` |
| Guarda del barandal | `ds_left < 2 m` | dejar de desviarse → pasar a enderezar |
| PASS (pasar/enderezar) | resto | `steer = K_HEAD * heading_dev` (vuelve al rumbo) |

**Anti-360:** la desviación a la izquierda está acotada por `DEV_TARGET` (giroscopio),
por lo que el vehículo nunca puede dar la vuelta completa.

**Reincorporación:** cuando el costado derecho queda libre (`rm,rr ≥ 4 m` tras haber
visto el bus) y el rumbo está recto (`|heading - saved| < 0.12`), se reinicia el
integrador del PID y se reanuda el seguimiento de carril.

## Tabla de dispositivos

| Dispositivo | Nombre Webots | Configuración | Uso |
|-------------|---------------|---------------|-----|
| Cámara | `camera` | 256×128, FOV 1, BGRA, Recognition | Seguimiento de carril + reconocimiento del autobús |
| LiDAR | `Sick LMS 291` | FOV 180°, ±20 índices centrales, < 20 m | Distancia al autobús |
| Giroscopio | `gyro` | 3 ejes, rad/s (marco local) | Integración del heading; acota la desviación (anti-360) |
| DistanceSensor | `ds_right_front` | 1 rayo, 0–5 m | Costado derecho (detección del bus) |
| DistanceSensor | `ds_right_mid` | 1 rayo, 0–5 m | Costado derecho (rebase) |
| DistanceSensor | `ds_right_rear` | 1 rayo, 0–5 m | "Último sensor": confirma que el bus quedó atrás |
| DistanceSensor | `ds_left` | 1 rayo, 0–8 m | Barandal izquierdo (limita la desviación) |

## Parámetros clave

| Parámetro | Valor | Significado |
|-----------|-------|-------------|
| `APPROACH_DIST` | 16.0 m | bus reconocido + LiDAR < esto → inicia evasión |
| `EMERGENCY_DIST` | 4.5 m | bus muy cerca de frente → freno de seguridad |
| `DEV_TARGET` | 0.55 rad | desviación máxima al salir (anti-360, ~32°) |
| `LEFT_LIMIT` | 2.0 m | `ds_left` < esto → no desviarse más (barandal) |
| `WALL_CLEAR_DIST` | 4.0 m | sensor lateral "libre" (sin obstáculo) |
| `K_HEAD` | 1.20 | ganancia para enderezar / volver al rumbo |
| `HEADING_TOLERANCE` | 0.12 rad | tolerancia de rumbo recto para reincorporarse |
| `EVADE_SPEED` | 10 km/h | velocidad durante la evasión |
