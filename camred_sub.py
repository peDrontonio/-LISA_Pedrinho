#!/usr/bin/python3
# Nodo ROS para receber imagens do tópico /Imagens,
# reduzir a resolução e publicar no tópico /Imagens_Reduzidas

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

def image_callback(msg):
    try:
        # Converte a mensagem de imagem ROS para uma imagem OpenCV
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Obtem o tamanho original da imagem
        original_size = cv_image.shape[1], cv_image.shape[0]

        # Reduz a resolução da imagem (por exemplo, para 1/2 da original)
        reduced_resolution = cv2.resize(cv_image, (0, 0), fx=0.5, fy=0.5, interpolation=cv2.INTER_LINEAR)
        
        # Redimensiona a imagem de volta para o tamanho original
        resized_image = cv2.resize(reduced_resolution, original_size, interpolation=cv2.INTER_NEAREST)
        
        # Converte a imagem OpenCV de volta para uma mensagem de imagem ROS
        reduced_msg = bridge.cv2_to_imgmsg(resized_image, "bgr8")
        
        # Publica a imagem reduzida
        pub.publish(reduced_msg)
        
        # Exibe a imagem reduzida
        cv2.imshow("Imagem Reduzida", resized_image)
        cv2.waitKey(1)
        
    except CvBridgeError as e:
        rospy.logerr("Erro ao converter imagem: %s" % e)

# Inicializa o nó ROS
rospy.init_node('Reduzir_Imagem', anonymous=False)

# Cria um objeto CvBridge para conversão entre ROS e OpenCV
bridge = CvBridge()

# Cria um publicador para o tópico /Imagens_Reduzidas
pub = rospy.Publisher('/Imagens_Reduzidas', Image, queue_size=10)

# Cria um assinante para o tópico /Imagens
rospy.Subscriber('/Imagens', Image, image_callback)

# Mantém o nó em execução
rospy.spin()

# Libera todas as janelas OpenCV
cv2.destroyAllWindows()
