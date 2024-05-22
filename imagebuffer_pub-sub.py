#!/usr/bin/python3
# Nodo ROS para receber imagens do tópico /Imagens,
# armazenar em um buffer e publicar periodicamente
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from collections import deque

class ImageBufferNode:
    def __init__(self):
        # Inicializa o nó ROS
        rospy.init_node('buffer_imagens', anonymous=False)

        # Cria um objeto CvBridge para conversão entre ROS e OpenCV
        self.bridge = CvBridge()

        # Cria um publicador para o tópico /Imagens_Bufferizadas
        self.pub = rospy.Publisher('/Imagens_Bufferizadas', Image, queue_size=10)

        # Cria um assinante para o tópico /Imagens
        self.sub = rospy.Subscriber('/Imagens', Image, self.image_callback)

        # Buffer para armazenar as imagens
        self.buffer = deque(maxlen=10)  # Buffer com capacidade para 10 imagens

        # Contador de frames
        self.frame_count = 0

    def image_callback(self, msg):
        try:
            # Converte a mensagem de imagem ROS para uma imagem OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Adiciona a imagem ao buffer
            self.buffer.append(cv_image)
            
            # Incrementa o contador de frames
            self.frame_count += 1
            
            # Publica uma imagem do buffer a cada 10 frames
            if self.frame_count % 10 == 0:
                if self.buffer:
                    buffered_image = self.buffer.popleft()
                    
                    # Converte a imagem OpenCV de volta para uma mensagem de imagem ROS
                    buffered_msg = self.bridge.cv2_to_imgmsg(buffered_image, "bgr8")
                    
                    # Publica a imagem
                    self.pub.publish(buffered_msg)
                    
                    # Exibe a imagem publicada
                    cv2.imshow("Imagem Bufferizada", buffered_image)
                    cv2.waitKey(1)
        
        except CvBridgeError as e:
            rospy.logerr("Erro ao converter imagem: %s" % e)

if __name__ == '__main__':
    try:
        # Cria uma instância do nó buffer
        image_buffer_node = ImageBufferNode()
        
        # Mantém o nó em execução
        rospy.spin()
        
        # Libera todas as janelas OpenCV
        cv2.destroyAllWindows()
        
    except rospy.ROSInterruptException:
        pass
