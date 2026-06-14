"""
===============================================================================
  Actividad 4.2 — Evasion de Obstaculos (MR4010.10)
  Controlador Webots: PID Lane Following + Recognition + LiDAR + Gyro + Wall Following
===============================================================================

  Descripcion:
      Controlador autonomo para simulacion en Webots que incorpora un algoritmo
      de seguimiento de pared derecha como mecanismo de evasion de obstaculos,
      construido sobre el seguidor de linea con PID de la Actividad 2.1.

  Pipeline de vision (seguimiento de carril, reutilizado de la Act. 2.1 / 3.1):
      Camara -> Grises -> Canny(50,150) -> ROI trapezoidal -> Hough -> Error -> EMA -> PID

  Mecanismo de evasion (Actividad 4.2):
      1. El nodo Recognition de la camara identifica el autobus al frente
         (mensaje de consola).
      2. El LiDAR (Sick LMS 291) indica la distancia al autobus (mensaje de consola).
      3. Al cruzar el umbral de distancia el controlador deja de seguir la linea
         y lee el giroscopio para guardar la orientacion (angulo en el eje z).
      4. El vehiculo maniobra con seguimiento de pared derecha leyendo 3 sensores
         de distancia de un solo rayo en el costado derecho.
      5. El algoritmo termina cuando el ultimo sensor (trasero-derecho) ya no ve
         obstaculo; el vehiculo avanza, recupera la orientacion previa con el
         giroscopio y reanuda el seguimiento de linea.

  Maquina de estados:
      LINE_FOLLOW -> APPROACH -> SAVE_HEADING -> WALL_FOLLOW_RIGHT -> RECOVER_HEADING -> LINE_FOLLOW

  Ganancias PID:  Kp=0.008 | Ki=0.0 | Kd=0.015   (reutilizadas de la Act. 2.1)
  Suavizado EMA:  alpha=0.6

  Equipo:
      Antonio Olvera Donlucas          A01795617
      Carlos Monir Radovich Saad       A01797569
      Andres Roberto Osuna Gonzalez    A01796264
      Oscar Alberto Ramirez Anaya      A01795438

  Institucion:
      Instituto Tecnologico y de Estudios Superiores de Monterrey
      Maestria en Inteligencia Artificial

  Fecha: Junio 2026
===============================================================================
"""

import math
import numpy as np
import cv2
import traceback

# Imports de Webots
from controller import Display, Keyboard
from vehicle import Driver


# ============================================================
# 1. CONSTANTES
# ============================================================

# --- Velocidad y limites (reutilizados de la Act. 2.1 / 3.1) ---
TARGET_SPEED = 30           # km/h — velocidad crucero en seguimiento de carril
MAX_SPEED = 250             # km/h — limite de velocidad
MAX_ANGLE = 0.5             # radianes — angulo maximo del volante

# --- Ganancias del control PID (mantienen al carro centrado en el carril) ---
KP = 0.008                  # Proporcional
KI = 0.0                    # Integral (no usada: causaba oscilaciones)
KD = 0.015                  # Derivativo (suaviza el giro)

# --- Configuracion del LiDAR (Sick LMS 291) ---
LIDAR_HALF_AREA = 20        # indices a cada lado del centro que revisamos
LIDAR_MAX_DIST = 20.0       # metros — el LiDAR ignora cualquier cosa mas alla

# --- Estados de la maquina de evasion ---
STATE_LINE_FOLLOW  = "LINE_FOLLOW"        # seguimiento de carril con PID
STATE_APPROACH     = "APPROACH"           # autobus reconocido, acercandose
STATE_SAVE_HEADING = "SAVE_HEADING"       # guardar orientacion y frenar
STATE_WALL_FOLLOW  = "WALL_FOLLOW_RIGHT"  # seguimiento de pared derecha
STATE_RECOVER      = "RECOVER_HEADING"    # recuperar orientacion previa

# --- Umbrales de evasion ---
APPROACH_DIST    = 15.0     # m: LiDAR ve el autobus Y la camara lo reconoce -> APPROACH
BRAKE_DIST       =  8.0     # m: LiDAR < esto -> guardar heading y arrancar maniobra
TARGET_WALL_DIST =  1.5     # m: brecha deseada entre el costado derecho y el autobus
CORNER_THRESHOLD =  1.2     # m: ds_right_front < esto -> giro fuerte a la izquierda
WALL_CLEAR_DIST  =  4.0     # m: sensor >= esto -> sin obstaculo a la derecha
K_WALL           =  0.25    # ganancia proporcional: error de brecha -> direccion (rad)
MIN_MANEUVER_STEPS = 50     # pasos minimos antes de evaluar la salida por sensor trasero

# --- Velocidades por estado ---
APPROACH_SPEED   = 10.0     # km/h acercandose al autobus
WALL_SPEED       = 12.0     # km/h durante la maniobra de pared
RECOVER_SPEED    = 20.0     # km/h mientras re-alinea la orientacion

# --- Recuperacion de orientacion (giroscopio) ---
K_RECOVER        =  0.80    # ganancia: error de heading (rad) -> direccion (rad)
HEADING_TOLERANCE=  0.08    # rad: |delta heading| < esto -> reanudar carril

# --- Debug ---
DEBUG_EVERY      = 30       # imprimir info cada N pasos


# ============================================================
# 2. SEGUIMIENTO DE LINEAS (pipeline Canny + Hough + PID)
#    Reutilizado sin cambios de la Actividad 2.1 / 3.1
# ============================================================

def get_image(camera):
    """Extrae la imagen de la camara y la convierte a una matriz Numpy (BGRA)."""
    raw = camera.getImage()
    if raw is None:
        return None
    return np.frombuffer(raw, np.uint8).reshape(
        (camera.getHeight(), camera.getWidth(), 4)
    )


def procesar_lineas(image):
    """
    Convierte a grises, aplica Canny, recorta la region de interes (ROI) de la
    carretera y busca lineas con la transformada de Hough.
    Retorna la imagen en grises y las lineas detectadas.
    """
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blur, 50, 150)

    # ROI trapezoidal: la zona de la carretera esta en la mitad inferior
    h, w = edges.shape
    mask = np.zeros_like(edges)
    polygon = np.array([[
        (0, h),                         # esquina inferior izquierda
        (int(0.2 * w), int(0.5 * h)),   # superior izquierda
        (int(0.8 * w), int(0.5 * h)),   # superior derecha
        (w, h),                         # esquina inferior derecha
    ]], dtype=np.int32)
    cv2.fillPoly(mask, polygon, 255)
    masked_edges = cv2.bitwise_and(edges, mask)

    lines = cv2.HoughLinesP(
        masked_edges,
        rho=1,
        theta=np.pi / 180,
        threshold=15,
        minLineLength=8,
        maxLineGap=5,
    )
    return gray, lines


def calcular_error_direccion(lines, setpoint):
    """
    Calcula que tan lejos esta el centro del carril respecto al centro de la
    camara (setpoint). Filtra lineas casi horizontales y toma la mas cercana
    al centro. Retorna None si no hay lineas validas.
    """
    if lines is None:
        return None
    candidates = []
    for line in lines:
        x1, y1, x2, y2 = line[0]
        if abs(x2 - x1) > 3 * abs(y2 - y1):   # descartar horizontales
            continue
        mid_x = (x1 + x2) / 2.0
        candidates.append(mid_x - setpoint)
    if not candidates:
        return None
    return min(candidates, key=abs)


# ============================================================
# 3. LIDAR Y RECONOCIMIENTO DE AUTOBUS
# ============================================================

def process_lidar(lidar):
    """
    Procesa el LiDAR Sick LMS 291 revisando solo la region central
    (+/-LIDAR_HALF_AREA indices). Reutilizado de la Act. 3.1.
    Retorna (angulo, distancia) del obstaculo mas cercano, o (None, None).
    """
    range_data = lidar.getRangeImage()
    if not range_data:
        return None, None

    n = len(range_data)
    center = n // 2
    sumx = 0
    collision_count = 0
    obstacle_dist = 0.0

    for x in range(center - LIDAR_HALF_AREA, center + LIDAR_HALF_AREA):
        r = range_data[x]
        if r <= LIDAR_MAX_DIST and not math.isinf(r) and not math.isnan(r):
            sumx += x
            collision_count += 1
            obstacle_dist += r

    if collision_count == 0:
        return None, None

    avg_angle = (sumx / collision_count / n - 0.5) * lidar.getFov()
    avg_dist = obstacle_dist / collision_count
    return avg_angle, avg_dist


def detect_bus_ahead(camera):
    """
    Recorre los objetos del nodo Recognition de la camara y busca un autobus
    (model == 'autobus') en el tercio central de la imagen (para descartar
    autobuses perifericos que no bloquean el carril).

    Retorna (True, posicion_en_imagen, colores) o (False, None, None).
    """
    objects = camera.getRecognitionObjects()
    cam_cx = camera.getWidth() / 2.0

    for obj in objects:
        # getModel() devuelve el campo 'model' del Solid reconocido
        if obj.getModel() == "autobus":
            pos = obj.getPositionOnImage()   # [pixel_x, pixel_y]
            colors = obj.getColors()         # [r, g, b, ...]
            if abs(pos[0] - cam_cx) < cam_cx * 0.5:
                return True, pos, colors
    return False, None, None


# ============================================================
# 4. MAIN — BUCLE PRINCIPAL DEL CONTROLADOR
# ============================================================

def main():
    # --- Inicializacion de Webots ---
    driver = Driver()
    timestep = int(driver.getBasicTimeStep())

    # Camara + nodo Recognition (para reconocer el autobus)
    camera = driver.getDevice("camera")
    camera.enable(timestep)
    camera.enableRecognition(timestep)
    cam_width = camera.getWidth()
    cam_height = camera.getHeight()
    setpoint = cam_width / 2.0

    # LiDAR (Sick LMS 291)
    lidar = driver.getDevice("Sick LMS 291")
    lidar.enable(timestep)
    lidar.enablePointCloud()

    # Giroscopio (para guardar / recuperar la orientacion en z)
    gyro = driver.getDevice("gyro")
    gyro.enable(timestep)

    # Sensores de distancia de un solo rayo en el costado derecho
    ds_right_front = driver.getDevice("ds_right_front")
    ds_right_mid = driver.getDevice("ds_right_mid")
    ds_right_rear = driver.getDevice("ds_right_rear")
    for ds in (ds_right_front, ds_right_mid, ds_right_rear):
        ds.enable(timestep)

    # Teclado para controles manuales de velocidad
    keyboard = Keyboard()
    keyboard.enable(timestep)

    # --- Variables de control ---
    speed = TARGET_SPEED
    vehicle_state = STATE_LINE_FOLLOW
    prev_error = 0.0
    integral = 0.0
    smoothed_error = None

    heading = 0.0           # yaw acumulado (integral del giroscopio en z)
    saved_heading = 0.0     # orientacion guardada al iniciar la evasion
    wall_found = False      # ya se "engancho" la cara izquierda del autobus
    maneuver_steps = 0      # pasos transcurridos dentro de la maniobra
    step_count = 0

    print("[INFO] === Actividad 4.2 — Evasion de Obstaculos ===")
    print(f"[INFO] Camara {cam_width}x{cam_height} -> setpoint = {setpoint}")
    print("[INFO] Reconocimiento de camara activado")
    print("[INFO] LiDAR Sick LMS 291 activado")
    print("[INFO] Giroscopio activado")
    print("[INFO] Sensores laterales derecha (front/mid/rear) activados")
    print(f"[INFO] PID: Kp={KP} Ki={KI} Kd={KD} | Velocidad crucero: {TARGET_SPEED} km/h")
    print("[INFO] Simulacion iniciada correctamente.")

    # --------------------------------------------------------
    # CICLO PRINCIPAL DE SIMULACION
    # --------------------------------------------------------
    while driver.step() != -1:
        try:
            step_count += 1

            # ====================================================
            # PASO 1: LEER CAMARA Y BUSCAR LINEAS DEL CARRIL
            # ====================================================
            image = get_image(camera)
            if image is None:
                continue
            gray, lines = procesar_lineas(image)
            raw_error = calcular_error_direccion(lines, setpoint)

            if raw_error is not None:
                if smoothed_error is None:
                    smoothed_error = raw_error
                else:
                    smoothed_error = 0.6 * smoothed_error + 0.4 * raw_error
            else:
                smoothed_error = None

            # ====================================================
            # PASO 2: CONTROL PID (direccion de seguimiento de carril)
            # ====================================================
            if smoothed_error is not None:
                integral += smoothed_error
                derivative = smoothed_error - prev_error
                pid_steering = (KP * smoothed_error) + (KI * integral) + (KD * derivative)
                prev_error = smoothed_error
                pid_steering = max(-MAX_ANGLE, min(MAX_ANGLE, pid_steering))
            else:
                pid_steering = 0.0
                integral = 0.0
                prev_error = 0.0

            # ====================================================
            # PASO 3: LIDAR (distancia al obstaculo frontal)
            # ====================================================
            _, lidar_dist = process_lidar(lidar)

            # ====================================================
            # PASO 4: GIROSCOPIO (integrar yaw en z, siempre)
            # ====================================================
            # getValues() -> [wx, wy, wz] en rad/s (marco local del vehiculo)
            # wz > 0 = giro a la izquierda; wz < 0 = giro a la derecha
            heading += gyro.getValues()[2] * (timestep / 1000.0)

            # ====================================================
            # PASO 5: RECONOCIMIENTO DEL AUTOBUS (camara Recognition)
            # ====================================================
            bus_recognized, bus_pos, bus_colors = detect_bus_ahead(camera)

            # ====================================================
            # PASO 6: MAQUINA DE ESTADOS DE EVASION
            # ====================================================
            rf = ds_right_front.getValue()
            rm = ds_right_mid.getValue()
            rr = ds_right_rear.getValue()

            if vehicle_state == STATE_LINE_FOLLOW:
                # Seguimiento normal de carril con PID
                driver.setSteeringAngle(pid_steering)
                driver.setCruisingSpeed(speed)
                # Transicion: autobus reconocido y dentro del rango de aproximacion
                if bus_recognized and lidar_dist is not None and lidar_dist < APPROACH_DIST:
                    vehicle_state = STATE_APPROACH

            elif vehicle_state == STATE_APPROACH:
                # Seguir el carril pero reducir velocidad y vigilar la distancia
                driver.setSteeringAngle(pid_steering)
                driver.setCruisingSpeed(APPROACH_SPEED)
                if lidar_dist is not None and lidar_dist < BRAKE_DIST:
                    vehicle_state = STATE_SAVE_HEADING
                elif not bus_recognized and (lidar_dist is None or lidar_dist > APPROACH_DIST):
                    # Falsa alarma: el autobus ya no esta al frente
                    vehicle_state = STATE_LINE_FOLLOW

            elif vehicle_state == STATE_SAVE_HEADING:
                # Guardar la orientacion actual y frenar (transicion de un paso)
                saved_heading = heading
                wall_found = False
                maneuver_steps = 0
                driver.setCruisingSpeed(0)
                driver.setBrakeIntensity(1.0)
                print(f"[SAVE_HEADING] Orientacion guardada = {saved_heading:.4f} rad — iniciando evasion")
                vehicle_state = STATE_WALL_FOLLOW

            elif vehicle_state == STATE_WALL_FOLLOW:
                # Seguimiento de pared derecha con los 3 sensores laterales
                driver.setBrakeIntensity(0.0)
                driver.setCruisingSpeed(WALL_SPEED)
                maneuver_steps += 1

                if rm < WALL_CLEAR_DIST:
                    wall_found = True   # ya vemos la cara izquierda del autobus

                if not wall_found:
                    # Fase 1: buscar el obstaculo girando a la izquierda
                    wall_steering = -0.40
                elif rf < CORNER_THRESHOLD:
                    # Fase 2: esquina muy cerca al frente-derecha -> giro fuerte izquierda
                    wall_steering = -MAX_ANGLE
                else:
                    # Fase 3: control de brecha (mantener rm ~ TARGET_WALL_DIST)
                    # error > 0 (muy lejos del bus) -> girar derecha (+)
                    # error < 0 (muy cerca del bus) -> girar izquierda (-)
                    gap_error = rm - TARGET_WALL_DIST
                    wall_steering = K_WALL * gap_error
                    wall_steering = max(-MAX_ANGLE, min(MAX_ANGLE, wall_steering))

                driver.setSteeringAngle(wall_steering)

                # Salida: ya enganchamos el bus, el sensor trasero esta libre
                # y transcurrieron suficientes pasos
                if wall_found and rr >= WALL_CLEAR_DIST and maneuver_steps > MIN_MANEUVER_STEPS:
                    print("[WALL_FOLLOW] Obstaculo superado — iniciando recuperacion de orientacion")
                    vehicle_state = STATE_RECOVER

            elif vehicle_state == STATE_RECOVER:
                # Recuperar la orientacion previa a la evasion con el giroscopio
                heading_error = heading - saved_heading   # >0: giro neto a la izquierda
                recover_steering = K_RECOVER * heading_error   # corregir girando a la derecha (+)
                recover_steering = max(-MAX_ANGLE, min(MAX_ANGLE, recover_steering))
                driver.setSteeringAngle(recover_steering)
                driver.setCruisingSpeed(RECOVER_SPEED)

                if abs(heading_error) < HEADING_TOLERANCE and lidar_dist is None:
                    integral = 0.0
                    prev_error = 0.0
                    smoothed_error = None
                    print("[RECOVER] Orientacion recuperada — reanudando seguimiento de carril")
                    vehicle_state = STATE_LINE_FOLLOW

            # ====================================================
            # PASO 7: MENSAJES DE CONSOLA (evidencia de la rubrica)
            # ====================================================
            if bus_recognized and bus_pos is not None and step_count % 10 == 0:
                print(f"[RECOGNITION] Autobus detectado | pos_imagen=({int(bus_pos[0])},{int(bus_pos[1])}) "
                      f"| color=({bus_colors[0]:.2f},{bus_colors[1]:.2f},{bus_colors[2]:.2f})")

            if step_count % DEBUG_EVERY == 0:
                dist_str = f"{lidar_dist:.1f}m" if lidar_dist is not None else "---"
                if vehicle_state == STATE_APPROACH:
                    print(f"[APPROACH] Autobus al frente | LiDAR={dist_str}")
                print(f"[{vehicle_state}] LiDAR={dist_str} heading={heading:.3f} "
                      f"saved={saved_heading:.3f} | rf={rf:.2f} rm={rm:.2f} rr={rr:.2f}")

            # ====================================================
            # CONTROLES MANUALES DE VELOCIDAD
            # ====================================================
            key = keyboard.getKey()
            if key == keyboard.UP and speed < MAX_SPEED:
                speed += 5
            elif key == keyboard.DOWN and speed >= 5:
                speed -= 5

        except Exception as e:
            print(f"[ERROR] {e}")
            traceback.print_exc()
            break


if __name__ == "__main__":
    main()
