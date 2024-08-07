#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger, TriggerRequest
from std_msgs.msg import String, Float32, Bool

class Controlador:
    def __init__(self):
        rospy.init_node('controlador_node')
        rospy.wait_for_service('/get_finger_count')
        rospy.wait_for_service('/recognize_gesture')
        rospy.wait_for_service('/recognize_face')

        self.get_finger_count = rospy.ServiceProxy('/get_finger_count', Trigger)
        self.recognize_gesture = rospy.ServiceProxy('/recognize_gesture', Trigger)
        self.recognize_face = rospy.ServiceProxy('/recognize_face', Trigger)
        self.image_sub = rospy.Subscriber('/Imagens', Image, self.image_callback)
        self.distance_sub = rospy.Subscriber('/face_distance', Float32, self.distance_callback)
        self.result_pub = rospy.Publisher('/resultados', String, queue_size=10)
        self.position_pub = rospy.Publisher('/position', Bool, queue_size = 10)
        self.servo_control = rospy.Subscriber('/position', Bool, queue_size = 10)
        self.position = False
        self.current_image = None
        self.face_distance = None
        self.stop_counting = False
        self.gesture_active = False
        self.face_active = False
        self.rate = rospy.Rate(1)  # 1 Hz

    def image_callback(self, msg):
        self.current_image = msg

    def distance_callback(self, msg):
        self.face_distance = msg.data

    def run(self):
        while not rospy.is_shutdown():
            if self.current_image is None:
                rospy.loginfo("Aguardando imagem...")
                self.rate.sleep()
                continue

            if self.stop_counting:
                if self.gesture_active:
                    rospy.loginfo("Reconhecimento de gestos em andamento...")
                    try:
                        gesture_response = self.recognize_gesture()
                        if gesture_response.success:
                            rospy.loginfo(f"Gesto reconhecido: {gesture_response.message}")
                            if "3 vezes seguidas" in gesture_response.message:
                                rospy.loginfo("Gesto reconhecido 3 vezes. Reativando contador de dedos.")
                                self.result_pub.publish(f"Gesto reconhecido: {gesture_response.message}")
                                self.gesture_active = False
                                self.stop_counting = False
                                rospy.set_param('/stop_counting', False)
                        else:
                            rospy.loginfo("Nenhum gesto reconhecido ou reconhecimento não concluído.")
                    except rospy.ServiceException as e:
                        rospy.logerr("Falha ao chamar o serviço: %s", e)
                    self.rate.sleep()
                    continue
                
                if self.face_active:
                    rospy.loginfo("Reconhecimento de rostos em andamento...")
                    try:
                        face_response = self.recognize_face()
                        if face_response.success:
                            rospy.loginfo(f"Rosto reconhecido: {face_response.message}")
                            self.result_pub.publish(f"Rosto reconhecido: {face_response.message}")
                            self.face_active = False
                            self.stop_counting = False
                            rospy.set_param('/stop_counting', False)
                        else:
                            rospy.loginfo("Nenhum rosto reconhecido ou reconhecimento não concluído.")
                    except rospy.ServiceException as e:
                        rospy.logerr("Falha ao chamar o serviço: %s", e)
                    self.rate.sleep()
                    continue
                if self.position:
                    rospy.loginfo("Ativando os servos...")
                    self.position_pub.publish(True)
                    self.position = False  # Reinicia a variável position após a publicação
                    rospy.set_param('/position', False)                    
                    self.stop_counting = False
                    rospy.set_param('/stop_counting', False)
                    continue
                    
            try:
                response = self.get_finger_count()
                if response.success:
                    finger_count = int(response.message)
                    if finger_count == 2:
                        if self.face_distance is not None and self.face_distance < 1.0:
                            rospy.loginfo("Contador é 2 e rosto está a menos de 1 metro, ativando serviço de gestos.")
                            self.gesture_active = True
                            self.stop_counting = True
                            rospy.set_param('/stop_counting', True)
                        else:
                            rospy.loginfo("Contador é 2, mas rosto está a mais de 1 metro.")
                    elif finger_count == 3:
                        rospy.loginfo("Contador é 3, ativando serviço de reconhecimento de rostos.")
                        self.face_active = True
                        self.stop_counting = True
                        rospy.set_param('/stop_counting', True)
                    elif finger_count == 5:
                        rospy.loginfo("Contador é 5, ativando servos")
                        self.position = True
                        self.stop_counting = True
                        rospy.set_param('/stop_counting', True)
                    else:
                        rospy.loginfo("Outros contador de dedos.")
                        self.result_pub.publish(f"Numero de dedos: {finger_count}")
                else:
                    rospy.loginfo("Falha ao obter contagem de dedos.")
            except rospy.ServiceException as e:
                rospy.logerr("Falha ao chamar o serviço: %s", e)
            
        rospy.sleep(2)  #Adiciona um delay de 2 segundos para uma nova identificação de gestos  
        self.rate.sleep()

if __name__ == '__main__':
    try:
        controlador = Controlador() 
        controlador.run()
    except rospy.ROSInterruptException:
        pass
