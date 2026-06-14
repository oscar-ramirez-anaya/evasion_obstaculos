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

## Pipeline 3 — Evasión por seguimiento de pared derecha

```
Giroscopio (eje z)
   -> heading += gyro.getValues()[2] * dt        (yaw integrado)

Sensores de distancia de un rayo (costado derecho)
   -> ds_right_front / ds_right_mid / ds_right_rear  (getValue, 0-5 m)
   -> ley de control de pared derecha               -> setSteeringAngle
                                                     -> setCruisingSpeed(WALL_SPEED)
```

## Máquina de estados

```
LINE_FOLLOW ──(bus reconocido && lidar < 15 m)──────────────> APPROACH
APPROACH ────(lidar < 8 m)──────────────────────────────────> SAVE_HEADING
SAVE_HEADING ─(inmediato: guarda heading, frena)────────────> WALL_FOLLOW_RIGHT
WALL_FOLLOW_RIGHT ─(wall_found && rr >= 4 m && steps > 50)───> RECOVER_HEADING
RECOVER_HEADING ──(|heading - saved| < 0.08 && lidar libre)─> LINE_FOLLOW
```

### Ley de control de pared derecha (3 fases)

| Fase | Condición | Acción de dirección |
|------|-----------|---------------------|
| Búsqueda | aún sin `wall_found` (`rm >= 4 m`) | girar izquierda `-0.40` rad |
| Esquina | `ds_right_front < 1.2 m` | giro fuerte izquierda `-MAX_ANGLE` |
| Brecha | resto | `K_WALL * (rm - 1.5)`, clamp ±MAX_ANGLE |

### Recuperación de orientación

`steer = K_RECOVER * (heading - saved_heading)`, clamp ±MAX_ANGLE. Al alinear
(`|error| < 0.08` y LiDAR libre) se reinicia el integrador del PID y se reanuda
el seguimiento de carril.

## Tabla de dispositivos

| Dispositivo | Nombre Webots | Configuración | Uso |
|-------------|---------------|---------------|-----|
| Cámara | `camera` | 256×128, FOV 1, BGRA, Recognition | Seguimiento de carril + reconocimiento del autobús |
| LiDAR | `Sick LMS 291` | FOV 180°, ±20 índices centrales, < 20 m | Distancia al autobús |
| Giroscopio | `gyro` | 3 ejes, rad/s (marco local) | Integración del heading en z |
| DistanceSensor | `ds_right_front` | 1 rayo, lookupTable 0–5 m lineal | Detección de esquina frontal-derecha |
| DistanceSensor | `ds_right_mid` | 1 rayo, lookupTable 0–5 m lineal | Control de brecha lateral |
| DistanceSensor | `ds_right_rear` | 1 rayo, lookupTable 0–5 m lineal | "Último sensor": fin de la maniobra |

## Parámetros clave

| Parámetro | Valor | Significado |
|-----------|-------|-------------|
| `APPROACH_DIST` | 15.0 m | LiDAR + reconocimiento disparan APPROACH |
| `BRAKE_DIST` | 8.0 m | Inicia la maniobra (guarda heading) |
| `TARGET_WALL_DIST` | 1.5 m | Brecha deseada al costado del autobús |
| `CORNER_THRESHOLD` | 1.2 m | Umbral de esquina frontal-derecha |
| `WALL_CLEAR_DIST` | 4.0 m | Sensor "libre" (sin obstáculo) |
| `K_WALL` | 0.25 | Ganancia de control de brecha |
| `K_RECOVER` | 0.80 | Ganancia de recuperación de heading |
| `HEADING_TOLERANCE` | 0.08 rad | Tolerancia de re-alineación |
| Velocidades | 30 / 10 / 12 / 20 km/h | LINE / APPROACH / WALL / RECOVER |
