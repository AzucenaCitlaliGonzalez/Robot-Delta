# NOTA: Este código permite grabar en vivo con la cámara calibrada

import pickle
import cv2
import os

# Abrir cámara
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)

# Establecer la resolución deseada
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640) #1920 
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480) #1080

# Tamaño de la imagen
h=480
w=640

# Ruta de guardado
save_path = r'C:\Users\rodri\OneDrive\Documentos\8vo_semestre\Temas_Selectos\Proyecto_robot_Delta\calibrar_camara\images_dst' #Asegurarse que la ruta no contenga espacios, vocales con acentos y "ñ"
# Crear la carpeta si no existe
os.makedirs(save_path, exist_ok=True)

# Read in the saved objpoints and imgpoints
cameraMatrix, dist = pickle.load(open( r"C:\Users\rodri\OneDrive\Documentos\8vo_semestre\Temas_Selectos\Proyecto_robot_Delta\calibrar_camara\resultados\calibration.pkl", "rb" ))

newCameraMatrix, roi = cv2.getOptimalNewCameraMatrix(cameraMatrix, dist, (w,h), 1, (w,h)) # Se utiliza para calcular una nueva matriz de la cámara optimizada
# - newCameraMatrix: Es la nueva matriz de la cámara. Se utiliza para el proceso de "undistortion" (corrección de la distorsión) y está optimizada para que las regiones de la imagen que se mantienen visibles sean lo más grandes posible.
# - roi: Es la Región de Interés (Region of Interest). Es una tupla (x, y, w, h) que define las coordenadas del rectángulo donde la imagen corregida no tiene píxeles negros. Este roi se usa para recortar la imagen corregida y eliminar los bordes vacíos, asegurando que solo se muestre el área de la imagen útil.

# Undistort with Remapping (método eficiente para videos)
mapx, mapy = cv2.initUndistortRectifyMap(cameraMatrix, dist, None, newCameraMatrix, (w,h), 5) # Esta función calcula las matrices de mapeo (mapx y mapy). Estas matrices contienen las coordenadas de píxeles que le dicen a cv.remap() de dónde tomar los píxeles de la imagen original para crear la nueva imagen sin distorsión.
num=0

while cap.isOpened():
# Read in an image
    success, img = cap.read()

    if not success:
        print("No se pudo leer la imagen de la cámara.")
        break
    

    dst = cv2.remap(img, mapx, mapy, cv2.INTER_LINEAR) # Esta función calcula las matrices de mapeo (mapx y mapy). Estas matrices contienen las coordenadas de píxeles que le dicen a cv.remap() de dónde tomar los píxeles de la imagen original para crear la nueva imagen sin distorsión.

    # Recortar la imagen al ROI válido
    x, y, roi_w, roi_h = roi
    dst = dst[y:y+roi_h, x:x+roi_w]
    cv2.imshow('Undistorted Image', dst)

    # Espera tecla por 5 ms
    k = cv2.waitKey(5)
    # Salir si se presiona 'q'
    if k == ord('q'):
        break

    # Guardar imagen si se presiona 's'
    elif k == ord('s'):
        filename = os.path.join(save_path, f'dst{num}.png')
        result = cv2.imwrite(filename, dst)

        if result:
            print(f"Imagen guardada: {filename}")
            num += 1
        else:
            print("Error al guardar la imagen.")

# ==========================
# Liberar recursos
# ==========================
cap.release()
cv2.destroyAllWindows()