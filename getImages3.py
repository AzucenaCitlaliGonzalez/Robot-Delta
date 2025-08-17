import cv2
import os

# Ruta de guardado
save_path = r'C:\Users\rodri\OneDrive\Documentos\8vo_semestre\Temas_Selectos\Proyecto_robot_Delta\calibrar_camara\images_chess' #Asegurarse que la ruta no contenga espacios, vocales con acentos y "ñ"

# Crear la carpeta si no existe
os.makedirs(save_path, exist_ok=True)

# Abrir cámara
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)

# Establecer la resolución deseada
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640) #1920 
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480) #1080

num = 0

while cap.isOpened():
    success, img = cap.read()
    
    if not success:
        print("No se pudo leer la imagen de la cámara.")
        break

    cv2.imshow('Img', img)

    # Espera tecla por 5 ms
    k = cv2.waitKey(5)

    # Salir si se presiona 'q'
    if k == ord('q'):
        break

    # Guardar imagen si se presiona 's'
    elif k == ord('s'):
        filename = os.path.join(save_path, f'img{num}.png')
        result = cv2.imwrite(filename, img)

        if result:
            print(f"Imagen guardada: {filename}")
            num += 1
        else:
            print("Error al guardar la imagen.")

# Liberar recursos
cap.release()
cv2.destroyAllWindows()
