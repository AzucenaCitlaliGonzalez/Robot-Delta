import numpy as np
import cv2 as cv
import glob
import pickle



################ ENCUENTRA LAS ESQUINAS DEL TABLERO DE AJEDREZ: PUNTOS DE OBJETO Y PUNTOS DE IMAGEN #############################

chessboardSize = (9,6) # Nnúmero de esquinas internas (interior corners) del tablero de ajedrez usado para calibrar. 9 esquinas internas en horizontal (columnas) y 6 esquinas internas en vertical (filas)
frameSize = (640,480) #(1920,1080) # Resolución de la cámara #(640,480)



# Criterio de finalización del algoritmo
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# "cv.TERM_CRITERIA_EPS" indica que el algoritmo terminara cuando el error sea menor a 0.001
# "cv.TERM_CRITERIA_MAX_ITER" indica que el algoritmo terminara cuando se haya ejecutado un máximo de 30 iteraciones
# "cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER" indica que el algoritmo terminara cuando se cumpla alguno de los dos criterios

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((chessboardSize[0] * chessboardSize[1], 3), np.float32) # Creamos una matriz de puntos 3D en el espacio real (sistema del tablero) para guardar las coordenadas de los vértices de los cuadrados. Por ejemplo: (chessboardSize[0] * chessboardSize[1], 3)=(9*6,3)=(54,3)
objp[:,:2] = np.mgrid[0:chessboardSize[0],0:chessboardSize[1]].T.reshape(-1,2) # Se toma las primeras dos coordenadas (x,y) de la matriz objp. "np.mgrid[0:chessboardSize[0],0:chessboardSize[1]]=np.mgrid[0:9, 0:6]" genera una cuadrícula con las coordenadas (x,y) de los vértices de los cuadrados del tablero.

size_of_chessboard_squares_mm = 29.8 # Tamaño de la arista de un cuadrado del tablero de ajedrez
objp = objp * size_of_chessboard_squares_mm # Se multiplica objp * size_of_chessboard_squares_mm para obtener las coordenadas de los vértices de los cuadrados del tablero de ajedrez.


# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

# Se accede a todas las fotos que se tomaron del tablero de ajedrez
images = glob.glob(r'C:\Users\rodri\OneDrive\Documentos\8vo_semestre\Temas_Selectos\Proyecto_robot_Delta\calibrar_camara\images_chess\*.png')

# Se usa un ciclo for para leer todas las fotos
for image in images:
    
    img = cv.imread(image) # Se lee la foto
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY) # Se convierte a escala de grises

    # Encuentra las esquinas del tablero de ajedrez
    ret, corners = cv.findChessboardCorners(gray, chessboardSize,None) # "corners" contiene las coordenadas de todas las esquinas internas detectadas

    # Si se encuentran, agregue puntos de objeto, puntos de imagen (después de refinarlos)
    if ret == True:
        print(image)
        objpoints.append(objp) # Va acumulando las coordenadas reales (3D) del tablero para cada imagen procesada
        corners2 = cv.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria) # Refina las coordenadas guardadas en corners para obtener una precisión subpíxel (valores decimales en lugar de solo enteros)
        imgpoints.append(corners) # Va acumulando las coordenadas de la fotos (2D) del tablero para cada imagen procesada

        # Draw and display the corners
        cv.drawChessboardCorners(img, chessboardSize, corners2, ret)
        cv.imshow('img', img)
        cv.waitKey(100)


cv.destroyAllWindows()




############## CALIBRACIÓN #######################################################
ret, cameraMatrix, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, frameSize, None, None) # Aquí se calibra la cámara. Se calcula los parámetros intrínsecos y extrínsecos de la cámara a partir de un conjunto de imágenes con un patrón conocido (tablero de ajedrez)
# - cameraMatrix: La matriz de la cámara o matriz intrínseca. Es una matriz de 3x3 que contiene los parámetros internos de la cámara, como la longitud focal (fx, fy) y el centro óptico (cx, cy).
# - dist: Los coeficientes de distorsión. Representan cómo la lente de la cámara distorsiona las imágenes (distorsión de barril o de alfiler). Sirve para corregir las imágenes y obtener una vista más real.
# - rvecs: Un vector de rotación para cada imagen, que describe la orientación del tablero de ajedrez en relación con la cámara.
# - tvecs: Un vector de traslación para cada imagen, que describe la posición del tablero de ajedrez en relación con la cámara.


# Save the camera calibration result for later use (we won't worry about rvecs / tvecs)
# La función dump del módulo pickle se usa para serializar (convertir un objeto de Python en una secuencia de bytes) un objeto y guardarlo en un archivo.
# Aquí la matriz de la cámara (cameraMatrix) y los coeficientes de distorsión (dist) se guardaran en un archivo llamado calibration.pkl
pickle.dump((cameraMatrix, dist), open( r"C:\Users\rodri\OneDrive\Documentos\8vo_semestre\Temas_Selectos\Proyecto_robot_Delta\calibrar_camara\resultados\calibration.pkl", "wb" ), protocol = 2)
# Aquí la matriz de la cámara (cameraMatrix) se guardaran en un archivo llamado cameraMatrix.pkl
pickle.dump(cameraMatrix, open( r"C:\Users\rodri\OneDrive\Documentos\8vo_semestre\Temas_Selectos\Proyecto_robot_Delta\calibrar_camara\resultados\cameraMatrix.pkl", "wb" ), protocol = 2)
# Aquí los coeficientes de distorsión (dist) se guardaran en un archivo llamado dist.pkl
pickle.dump(dist, open( r"C:\Users\rodri\OneDrive\Documentos\8vo_semestre\Temas_Selectos\Proyecto_robot_Delta\calibrar_camara\resultados\dist.pkl", "wb" ), protocol = 2)


############## UNDISTORTION #####################################################

img = cv.imread(r'C:\Users\rodri\OneDrive\Documentos\8vo_semestre\Temas_Selectos\Proyecto_robot_Delta\calibrar_camara\imagen_prueba.png')
h,  w = img.shape[:2] # Dimensiones de la imagen.
newCameraMatrix, roi = cv.getOptimalNewCameraMatrix(cameraMatrix, dist, (w,h), 1, (w,h)) # Se utiliza para calcular una nueva matriz de la cámara optimizada
# - newCameraMatrix: Es la nueva matriz de la cámara. Se utiliza para el proceso de "undistortion" (corrección de la distorsión) y está optimizada para que las regiones de la imagen que se mantienen visibles sean lo más grandes posible.
# - roi: Es la Región de Interés (Region of Interest). Es una tupla (x, y, w, h) que define las coordenadas del rectángulo donde la imagen corregida no tiene píxeles negros. Este roi se usa para recortar la imagen corregida y eliminar los bordes vacíos, asegurando que solo se muestre el área de la imagen útil.

# Undistort
dst = cv.undistort(img, cameraMatrix, dist, None, newCameraMatrix) # Aquí se corrige la distorsión de la imagen.


# crop the image
x, y, w, h = roi
print(roi)
dst = dst[y:y+h, x:x+w]
cv.imwrite(r'C:\Users\rodri\OneDrive\Documentos\8vo_semestre\Temas_Selectos\Proyecto_robot_Delta\calibrar_camara\caliResult1.png', dst)



# Undistort with Remapping (método eficiente para videos)
mapx, mapy = cv.initUndistortRectifyMap(cameraMatrix, dist, None, newCameraMatrix, (w,h), 5) # Esta función calcula las matrices de mapeo (mapx y mapy). Estas matrices contienen las coordenadas de píxeles que le dicen a cv.remap() de dónde tomar los píxeles de la imagen original para crear la nueva imagen sin distorsión.
dst = cv.remap(img, mapx, mapy, cv.INTER_LINEAR) # Esta función calcula las matrices de mapeo (mapx y mapy). Estas matrices contienen las coordenadas de píxeles que le dicen a cv.remap() de dónde tomar los píxeles de la imagen original para crear la nueva imagen sin distorsión.

# crop the image
x, y, w, h = roi
dst = dst[y:y+h, x:x+w]
cv.imwrite(r'C:\Users\rodri\OneDrive\Documentos\8vo_semestre\Temas_Selectos\Proyecto_robot_Delta\calibrar_camara\caliResult2.png', dst)




# Reprojection Error
mean_error = 0

for i in range(len(objpoints)):
    imgpoints2, _ = cv.projectPoints(objpoints[i], rvecs[i], tvecs[i], cameraMatrix, dist)
    error = cv.norm(imgpoints[i], imgpoints2, cv.NORM_L2)/len(imgpoints2)
    mean_error += error

print( "total error: {}".format(mean_error/len(objpoints)) )
