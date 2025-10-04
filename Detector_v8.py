# Detecta píldoras blancas y amarillas, corrige distorsión, calcula centro,
# convierte a coordenadas reales (cm->m) y las envía al ESP32 en METROS.
# Además: cuando detecta una pastilla, genera una trayectoria P0->P1->P2->P0
# y envía las coordenadas de la trayectoria cada DT_SEND segundos (por
# defecto 0.005 s = 5 ms). Durante la ejecución de la trayectoria
# el detector (YOLO) queda desactivado.

import os
from ultralytics import YOLO
import cv2
import numpy as np
import serial
import time
import pickle

# ============ Serial ============
esp32 = serial.Serial('COM3', 9600)  # Ajusta el puerto si hace falta
time.sleep(2)

# ============ Calibración/Lentes ============
h, w = 480, 640
chessboardSize = (9, 6)
cols, rows = 9, 6

cameraMatrix, dist = pickle.load(open(
    r"C:\Users\rodri\OneDrive\Documentos\8vo_semestre\Temas_Selectos\Proyecto_robot_Delta\calibrar_camara\resultados\calibration.pkl",
    "rb"
))

newCameraMatrix, roi = cv2.getOptimalNewCameraMatrix(cameraMatrix, dist, (w, h), 1, (w, h))
mapx, mapy = cv2.initUndistortRectifyMap(cameraMatrix, dist, None, newCameraMatrix, (w, h), cv2.CV_16SC2)

# ============ Modelo ============
ruta_modelo = os.path.join(os.path.dirname(__file__), "Modelo.pt")
print("Cargando modelo desde:", ruta_modelo)
model = YOLO(ruta_modelo)

# ============ Cámara ============
cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)  # Cambia si usas otra cámara

H = None  # Homografía (pix -> cm)
Z_DEFAULT_M = 0.30  # Altura de trabajo en METROS (30 cm)

# ============ Parámetros de trayectoria ============
P0 = np.array([0.0, 0.0, 0.2395])  # Home (m)
SEG_DUR = 5.0   # segundos por segmento (P0->P1, P1->P2, P2->P0). Ajustable.
DT_SEND = 0.005 # 5 ms entre envíos durante trayectoria
TRAJECTORY_CLASS = 9  # clase enviada durante trayectoria (opcional)

# función para generar Pd en el instante t (t desde 0 hasta 3*SEG_DUR)
def plan_tray(P0_local, P1_local, P2_local, t, seg_dur=SEG_DUR):
    tf1 = seg_dur
    tf2 = 2*seg_dur
    tf3 = 3*seg_dur

    if t <= tf1:
        # tramo P0->P1, interpolación lineal
        s = t / (tf1)  # 0..1
        Pd = P0_local + s * (P1_local - P0_local)
    elif t <= tf2:
        # tramo P1->P2
        s = (t - tf1) / (tf2 - tf1)
        Pd = P1_local + s * (P2_local - P1_local)
    else:
        # tramo P2->P0
        s = (t - tf2) / (tf3 - tf2)
        Pd = P2_local + s * (P0_local - P2_local)
    return Pd

# Bandera de ejecución de trayectoria
busy = False

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    frame = cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)

    # --- Homografía: calcular una sola vez ---
    if H is None:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ret_c, corners = cv2.findChessboardCorners(gray, chessboardSize, None)

        if ret_c:
            top_left     = corners[0][0]
            top_right    = corners[cols-1][0]
            bottom_left  = corners[(rows-1)*cols][0]
            bottom_right = corners[-1][0]

            pts_img = np.array([top_left, top_right, bottom_left, bottom_right], dtype=np.float32)

            # Puntos reales en cm (AJUSTA estas medidas al tablero real)
            pts_real = np.array([
                [0, 0],
                [23.84, 0],
                [0, 14.9],
                [23.84, 14.9]
            ], dtype=np.float32)

            H, _ = cv2.findHomography(pts_img, pts_real)

    envio_hecho = False

    if not busy:
        # --- Inferencia YOLO solo si no estamos ejecutando trayectoria ---
        results = model(frame, verbose=False)
        detected_point = None
        detected_label = None

        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])

                roi = frame[y1:y2, x1:x2]
                if roi.size == 0:
                    continue

                gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
                gray_blur = cv2.medianBlur(gray, 5)
                circles = cv2.HoughCircles(
                    gray_blur,
                    cv2.HOUGH_GRADIENT,
                    dp=1.2,
                    minDist=30,
                    param1=50,
                    param2=30,
                    minRadius=0,
                    maxRadius=0
                )

                if circles is None:
                    continue

                circles = np.uint16(np.around(circles))
                cx, cy, r = circles[0, 0]
                cx_abs = cx + x1
                cy_abs = cy + y1

                # Dibujo en frame
                cv2.circle(frame, (cx_abs, cy_abs), r, (0, 255, 0), 2)
                cv2.circle(frame, (cx_abs, cy_abs), 2, (0, 0, 255), 3)
                cv2.putText(frame, f"Centro:({cx_abs},{cy_abs}) R:{r}",
                            (x1, max(0, y2 + 25)), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (255, 0, 0), 2)

                if H is not None:
                    # pix -> cm
                    pixel_point = np.array([[[cx_abs, cy_abs]]], dtype=np.float32)
                    real_point = cv2.perspectiveTransform(pixel_point, H)
                    X_cm, Y_cm = real_point[0][0]

                    # cm -> m
                    X_m = X_cm / 100.0
                    Y_m = Y_cm / 100.0
                    Z_m = Z_DEFAULT_M

                    label = model.names[int(box.cls)]

                    # Asignar clase: 1=blanca, 2=amarilla, 0=otro
                    if label == "Pildora_blanca":
                        clase = 1
                    elif label == "Pildora_Amarilla":
                        clase = 2
                    else:
                        clase = 0

                    detected_point = np.array([X_m, Y_m, Z_m])
                    detected_label = clase

                    print(f"Detectado Pixel ({cx_abs},{cy_abs}) -> ({X_cm:.2f}cm,{Y_cm:.2f}cm)  clase:{clase}")
                else:
                    print(f"⚠️ Homografía no calculada aún. Pixel ({cx_abs},{cy_abs}) sin conversión.")

                # tomar solo la primera detección válida por frame
                break
            if detected_point is not None:
                break

        # Si detectamos una pastilla válida (clase 1 o 2), iniciar trayectoria
        if detected_point is not None and detected_label in (1, 2) and H is not None:
            # Definir P1 en metros (la detección)
            P1 = detected_point.copy()
            P2 = P1 + np.array([0.091, 0.0, 0.0])  # desplazar 9.1 cm en X (m)
            P3 = P0.copy()

            # Inicia la ejecución de la trayectoria en bucle bloqueante local: 
            # envia coordenadas cada DT_SEND y no ejecuta inferencia hasta terminar.
            print("Iniciando trayectoria: P0 -> P1 -> P2 -> P0")
            busy = True

            t_start = time.perf_counter()
            total_time = 3 * SEG_DUR
            t = 0.0
            next_send_time = t_start

            while t <= total_time + 1e-6:
                now = time.perf_counter()
                if now < next_send_time:
                    # esperamos activamente el siguiente tick (micro-sleep para alivio CPU)
                    time.sleep(max(0.0005, next_send_time - now))
                    continue

                t = now - t_start
                Pd = plan_tray(P0, P1, P2, t, seg_dur=SEG_DUR)
                # Enviar: X,Y,Z,CLASE\n (usamos TRAJECTORY_CLASS para indicarlo opcionalmente)
                mensaje = f"{Pd[0]:.4f},{Pd[1]:.4f},{Pd[2]:.4f},{TRAJECTORY_CLASS}\n"
                try:
                    esp32.write(mensaje.encode())
                except Exception as e:
                    print("Error enviando por serial:", e)

                # Opcional: dibujar punto de tray en la imagen (convertir a pix para mostrar)
                if H is not None:
                    # convert Pd (m) a cm, luego a pixel aproximado (inversa de H no trivial).
                    # Aquí solo dibujamos texto con la coordenada enviada.
                    cv2.putText(frame, f"TRAY: ({Pd[0]:.3f},{Pd[1]:.3f},{Pd[2]:.3f})",
                                (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)

                next_send_time += DT_SEND

            # al terminar, enviar la postura home final (asegurarse de llegar)
            mensaje = f"{P0[0]:.4f},{P0[1]:.4f},{P0[2]:.4f},{TRAJECTORY_CLASS}\n"
            esp32.write(mensaje.encode())
            print("Trayectoria completa. Volviendo a detección.")
            busy = False
            # tras terminar la trayectoria, continúa el loop principal y vuelve a detectar
            continue

        # Si no hubo detección válida, envía "sin objeto" (postura segura) periódicamente
        if detected_point is None and H is not None:
            mensaje = f"0.0000,0.0000,{Z_DEFAULT_M:.4f},0\n"
            esp32.write(mensaje.encode())

    else:
        # Si estamos en busy==True (esto normalmente no ocurre porque busy se gestiona
        # dentro del bloque anterior), podemos enviar "ocupado" o simplemente no inferir.
        pass

    # Visualización (si hubo inferencia, results existe; si no, mostramos frame tal cual)
    try:
        annot_frame = results[0].plot() if (not busy and 'results' in locals() and len(results)>0) else frame.copy()
    except Exception:
        annot_frame = frame.copy()

    Muestra_frame = cv2.addWeighted(annot_frame, 0.7, frame, 0.3, 0)
    cv2.imshow("YOLO + Círculo + Tray", Muestra_frame)

    # Esc para salir
    if cv2.waitKey(1) & 0xFF == 27:
        esp32.close()
        break

cap.release()
cv2.destroyAllWindows()
cv2.waitKey(1)
