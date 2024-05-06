#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2, CompressedPointCloud2
import sensor_msgs.point_cloud2 as pc2

def callback_point_cloud(msg):
    # Comprimir la nube de puntos recibida
    compressed_cloud_msg = CompressedPointCloud2()
    compressed_cloud_msg.header = msg.header
    compressed_cloud_msg.format = "pcl::PointCloud2"
    compressed_cloud_msg.data = pc2.compress(msg, compressed_cloud_msg.format)
    
    # Publicar la nube de puntos comprimida
    pub_compressed_point_cloud.publish(compressed_cloud_msg)
    
    rospy.loginfo("Nube de puntos comprimida publicada")

def main():
    # Inicializa el nodo suscriptor
    rospy.init_node('compressed_point_cloud_publisher', anonymous=True)

    # Suscríbete al tópico /zed2i/zed_node/point_cloud/cloud_registered
    rospy.Subscriber("/zed2i/zed_node/point_cloud/cloud_registered", PointCloud2, callback_point_cloud)
    
    # Mantiene el nodo en ejecución
    rospy.spin()

if __name__ == '__main__':
    try:
        # Inicializa el publicador de la nube de puntos comprimida
        pub_compressed_point_cloud = rospy.Publisher('/compressed_point_cloud', CompressedPointCloud2, queue_size=10)
        
        main()
    except rospy.ROSInterruptException:
        pass
