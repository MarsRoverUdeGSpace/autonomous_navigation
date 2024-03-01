#!/usr/bin/env python3
import rospy
import sys
import argparse
import numpy as np
import pyzed.sl as sl
from time import sleep
from sensor_msgs.msg import PointCloud2, PointField

# Camera resolution
def parse_args(init):
    if len(opt.input_svo_file)>0 and opt.input_svo_file.endswith(".svo"):
        init.set_from_svo_file(opt.input_svo_file)
        print("[Sample] Using SVO File input: {0}".format(opt.input_svo_file))
    elif len(opt.ip_address)>0 :
        ip_str = opt.ip_address
        if ip_str.replace(':','').replace('.','').isdigit() and len(ip_str.split('.'))==4 and len(ip_str.split(':'))==2:
            init.set_from_stream(ip_str.split(':')[0],int(ip_str.split(':')[1]))
            print("[Sample] Using Stream input, IP : ",ip_str)
        elif ip_str.replace(':','').replace('.','').isdigit() and len(ip_str.split('.'))==4:
            init.set_from_stream(ip_str)
            print("[Sample] Using Stream input, IP : ",ip_str)
        else :
            print("Unvalid IP format. Using live stream")
    if ("HD2K" in opt.resolution):
        init.camera_resolution = sl.RESOLUTION.HD2K
        print("[Sample] Using Camera in resolution HD2K")
    elif ("HD1200" in opt.resolution):
        init.camera_resolution = sl.RESOLUTION.HD1200
        print("[Sample] Using Camera in resolution HD1200")
    elif ("HD1080" in opt.resolution):
        init.camera_resolution = sl.RESOLUTION.HD1080
        print("[Sample] Using Camera in resolution HD1080")
    elif ("HD720" in opt.resolution):
        init.camera_resolution = sl.RESOLUTION.HD720
        print("[Sample] Using Camera in resolution HD720")
    elif ("SVGA" in opt.resolution):
        init.camera_resolution = sl.RESOLUTION.SVGA
        print("[Sample] Using Camera in resolution SVGA")
    elif ("VGA" in opt.resolution):
        init.camera_resolution = sl.RESOLUTION.VGA
        print("[Sample] Using Camera in resolution VGA")
    elif len(opt.resolution)>0: 
        print("[Sample] No valid resolution entered. Using default")
    else : 
        print("[Sample] Using default resolution")


def camera():
    global point_cloud
    global viewer
    global zed
    global res
    init = sl.InitParameters(depth_mode=sl.DEPTH_MODE.ULTRA,
                                 coordinate_units=sl.UNIT.METER,
                                 coordinate_system=sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP)
    parse_args(init)
    zed = sl.Camera()
    status = zed.open(init)
    if status != sl.ERROR_CODE.SUCCESS:
        print(repr(status))
        exit()
    if status != sl.ERROR_CODE.SUCCESS: #Ensure the camera has opened succesfully
        print("Camera Open : "+repr(status)+". Exit program.")
        exit()
    runtime = sl.RuntimeParameters()
    stream_params = sl.StreamingParameters()
    print("Streaming on port ",stream_params.port) #Get the port used to stream
    stream_params.codec = sl.STREAMING_CODEC.H264
    stream_params.bitrate = 4000
    status_streaming = zed.enable_streaming(stream_params) #Enable streaming
    if status_streaming != sl.ERROR_CODE.SUCCESS:
        print("Streaming initialization error: ", status_streaming)
        zed.close()
        exit()
    exit_app = False
    print(type(zed))

    res = sl.Resolution()
    res.width = 720
    res.height = 404

    camera_model = zed.get_camera_information().camera_model
    
    # Create OpenGL viewer
    #viewer = gl.GLViewer()
    #viewer.init(1, sys.argv, camera_model, res)

    #input("Hola")

    point_cloud = sl.Mat(res.width, res.height, sl.MAT_TYPE.F32_C4, sl.MEM.CPU)

# Creacion del mensaje de nube de puntos
def ndarray_to_point_cloud_msg(point_cloud_array):
    # Crear un mensaje de nube de puntos
    cloud_msg = PointCloud2()

    # Configurar la información de la nube de puntos
    cloud_msg.header.stamp = rospy.Time.now()
    cloud_msg.header.frame_id = 'base_link'  # Cambia 'base_link' según el marco de referencia de tu nube de puntos

    # Definir los campos de la nube de puntos
    cloud_msg.fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
    ]

    # Convertir los puntos a un formato adecuado para PointCloud2
    cloud_msg.data = np.array(point_cloud_array, dtype=np.float32).tobytes()

    # Configurar otros parámetros de la nube de puntos (opcional)
    cloud_msg.height = 404
    cloud_msg.width = 720
    cloud_msg.is_bigendian = False
    cloud_msg.point_step = 16  # Tamaño en bytes de cada punto (3 * 4 bytes para float32)
    cloud_msg.row_step = cloud_msg.point_step * 720

    return cloud_msg



if __name__ == "__main__":
    #iniciar nodo
    rospy.init_node('zed_node')

    parser = argparse.ArgumentParser()
    parser.add_argument('--input_svo_file', type=str, help='Path to an .svo file, if you want to replay it',default = '')
    parser.add_argument('--ip_address', type=str, help='IP Adress, in format a.b.c.d:port or a.b.c.d, if you have a streaming setup', default = '')
    parser.add_argument('--resolution', type=str, help='Resolution, can be either HD2K, HD1200, HD1080, HD720, SVGA or VGA', default = 'HD1080')
    opt = parser.parse_args()
    if len(opt.input_svo_file)>0 and len(opt.ip_address)>0:
        print("Specify only input_svo_file or ip_address, or none to use wired camera, not both. Exit program")
        exit()
    
    camera()
    print("Point cloud created")

    point_cloud_pub = rospy.Publisher('/point_cloud_topic', PointCloud2, queue_size=10)

    #while not rospy.is_shutdown() and viewer.is_available():
    while not rospy.is_shutdown():
        if zed.grab() == sl.ERROR_CODE.SUCCESS:
            zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA,sl.MEM.CPU, res)
            #viewer.updateData(point_cloud)
            point_cloud2 = point_cloud.get_data()
            
            cloud_msg = ndarray_to_point_cloud_msg(point_cloud2)
            point_cloud_pub.publish(cloud_msg)
        slp = rospy.Rate(60)
        slp.sleep()
    
    # disable Streaming
    zed.disable_streaming()
    #viewer.exit()
    zed.close()
    rospy.spin()