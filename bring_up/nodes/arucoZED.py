import pyzed.sl as sl
import cv2
import numpy as np

# Definir los parámetros de la cámara ZED
zed = sl.Camera()

init_params = sl.InitParameters()
init_params.camera_resolution = sl.RESOLUTION.HD720  # Resolución
init_params.camera_fps = 60  # FPS
init_params.coordinate_units = sl.UNIT.METER  # Se establecen metros como unidad de medida
#init_params.camera_buffer_count_linux = 1  # Buffers de la cámara
init_params.depth_mode = sl.DEPTH_MODE.NONE  # Modo de profundidad





# Definimos los parámetros de la cámara cv2
camera_matrix = np.array([[538.4284, 0, 320], [0, 538.4284, 240], [0, 0, 1]])
distortion = np.array([0, 0, 0, 0])

# Inicializar la cámara ZED
err = zed.open(init_params)
if err != sl.ERROR_CODE.SUCCESS:
    print("Error al abrir la cámara ZED: ", sl.ERROR_CODE.to_string(err))
    exit()


'''calibration_params = zed.get_camera_information().camera_configuration.calibration_parameters
# Focal length of the left eye in pixels
focal_left_x = calibration_params.left_cam.fx
print("fx: ",focal_left_x)
focal_left_y = calibration_params.left_cam.fy
print("fy ",focal_left_y)
# First radial distortion coefficient
k1 = calibration_params.left_cam.disto[0]
# Translation between left and right eye on z-axis
tz = calibration_params.T.z
# Horizontal field of view of the left eye in degrees
h_fov = calibration_params.left_cam.h_fov

'''


# Tipo de codigo ArUco a buscar
#aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
parameters = cv2.aruco.DetectorParameters()

# Configurar los parámetros de la cámara
zed_param = sl.RuntimeParameters()
zed_param.sensing_mode = sl.SENSING_MODE.STANDARD  # Normalon

while True:
    cv2.namedWindow('ArUco detection',cv2.WINDOW_NORMAL)

    cv2.setWindowProperty('ArUco detection',cv2.WND_PROP_FULLSCREEN,cv2.WINDOW_FULLSCREEN)
    if zed.grab(zed_param) == sl.ERROR_CODE.SUCCESS:
        # Obtener la imagen de la cámara ZED
        left_image = sl.Mat()
        zed.retrieve_image(left_image, sl.VIEW.LEFT)

        # Leer Frames la camara a una clase cv2
        zed_image = cv2.cvtColor(left_image.get_data(), cv2.COLOR_RGBA2RGB)

        # Convertir la imagen a escala de grises
        gray = cv2.cvtColor(zed_image, cv2.COLOR_BGR2GRAY)

        # Detectar los códigos ArUco en la imagen
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        
        # Dibujar los códigos ArUco y sus ejes de coordenadas en la imagen
        if len(corners) > 0:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.05, camera_matrix, distortion)
            cv2.aruco.drawDetectedMarkers(zed_image, corners, ids)
            print(tvecs[0][0])
            #cv2.aruco.drawDetectedAxes(zed_image, camera_matrix, distortion, rvecs, tvecs, 0.1)

        # Mostrar la imagen con los códigos ArUco y sus ejes de coordenadas
        cv2.imshow('ArUco detection', zed_image)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Liberar la cámara ZED y cerrar todas las ventanas
zed.close()
cv2.destroyAllWindows()
