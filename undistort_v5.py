import pickle
import cv2
import os
import numpy as np

# ==========================
# Configuración inicial
# ==========================
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

h, w = 480, 640
chessboardSize = (9, 6)
cols, rows = 9, 6

# Carpeta de guardado
save_path = r'C:\Users\rodri\OneDrive\Documentos\8vo_semestre\Temas_Selectos\Proyecto_robot_Delta\calibrar_camara\images_dst'
os.makedirs(save_path, exist_ok=True)

# Cargar calibración
cameraMatrix, dist = pickle.load(open(
    r"C:\Users\rodri\OneDrive\Documentos\8vo_semestre\Temas_Selectos\Proyecto_robot_Delta\calibrar_camara\resultados\calibration.pkl",
    "rb"
))

newCameraMatrix, roi = cv2.getOptimalNewCameraMatrix(cameraMatrix, dist, (w, h), 1, (w, h))
mapx, mapy = cv2.initUndistortRectifyMap(cameraMatrix, dist, None, newCameraMatrix, (w, h), cv2.CV_16SC2)

# ==========================
# Variables globales
# ==========================
H = None  # homografía (se calculará una vez)
num = 0

# ==========================
# Función para clics del mouse
# ==========================
def mouse_callback(event, x, y, flags, param):
    global H
    if event == cv2.EVENT_LBUTTONDOWN and H is not None:
        pixel_point = np.array([[[x, y]]], dtype=np.float32)
        real_point = cv2.perspectiveTransform(pixel_point, H)
        print(f"Pixel ({x},{y}) -> Coordenadas reales {real_point[0][0]}")

# Asignar callback
cv2.namedWindow("Undistorted Image")
cv2.setMouseCallback("Undistorted Image", mouse_callback)

# ==========================
# Loop principal
# ==========================
while cap.isOpened():
    ret, img = cap.read()
    if not ret:
        print("⚠️ No se pudo leer la cámara.")
        break

    dst = cv2.remap(img, mapx, mapy, cv2.INTER_LINEAR)

    # Detección de tablero SOLO si no tenemos homografía
    if H is None:
        gray = cv2.cvtColor(dst, cv2.COLOR_BGR2GRAY)
        ret_c, corners = cv2.findChessboardCorners(gray, chessboardSize, None)

        if ret_c:
            top_left     = corners[0][0]
            top_right    = corners[cols-1][0]
            bottom_left  = corners[(rows-1)*cols][0]
            bottom_right = corners[-1][0]

            # Puntos en la imagen
            pts_img = np.array([top_left, top_right, bottom_left, bottom_right], dtype=np.float32)

            # Puntos reales en cm (ajusta tus medidas aquí!)
            pts_real = np.array([
                [0, 0],
                [23.84, 0],
                [0, 14.9],
                [23.84, 14.9]
            ], dtype=np.float32)

            # Calcular homografía
            H, _ = cv2.findHomography(pts_img, pts_real)
            print("✅ Homografía calculada")

            # Dibujar esquinas en la imagen
            for pt in [top_left, top_right, bottom_left, bottom_right]:
                cv2.circle(dst, tuple(pt.astype(int)), 6, (0,255,0), -1)

    # Mostrar imagen
    cv2.imshow("Undistorted Image", dst)

    # Control de teclado
    k = cv2.waitKey(5) & 0xFF
    if k == ord('q'):
        break
    elif k == ord('s'):
        filename = os.path.join(save_path, f'undistorted_copy{num}.png')
        cv2.imwrite(filename, dst)
        print(f"💾 Imagen guardada: {filename}")
        num += 1

# ==========================
# Liberar recursos
# ==========================
cap.release()
cv2.destroyAllWindows()
