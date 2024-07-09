#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import mediapipe as mp
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger, TriggerResponse

BRANCO = (255, 255, 255)
PRETO = (0, 0, 0)
AZUL = (255, 0, 0)
VERDE = (0, 255, 0)
VERMELHO = (0, 0, 255)
AZUL_CLARO = (255, 255, 0)

mp_maos = mp.solutions.hands
mp_desenho = mp.solutions.drawing_utils
maos = mp_maos.Hands()

resolucao_x = 1280
resolucao_y = 720
bridge = CvBridge()
current_image = None
img_quadro = np.ones((resolucao_y, resolucao_x, 3), np.uint8) * 255
cor_pincel = (255, 0, 0)
espessura_pincel = 7
x_quadro, y_quadro = 0, 0
desenhando = False  # Variável para controlar se estamos desenhando ou não

def encontra_coordenadas_maos(img, lado_invertido=False):
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    resultado = maos.process(img_rgb)
    todas_maos = []
    if resultado.multi_hand_landmarks:
        for lado_mao, marcacoes_maos in zip(resultado.multi_handedness, resultado.multi_hand_landmarks):
            info_mao = {}
            coordenadas = []
            for marcacao in marcacoes_maos.landmark:
                coord_x, coord_y, coord_z = int(marcacao.x * resolucao_x), int(marcacao.y * resolucao_y), int(marcacao.z * resolucao_x)
                coordenadas.append((coord_x, coord_y, coord_z))

            info_mao['coordenadas'] = coordenadas
            if lado_invertido:
                if lado_mao.classification[0].label == 'Left':
                    info_mao['lado'] = 'Right'
                else:
                    info_mao['lado'] = 'Left'
            else:
                info_mao['lado'] = lado_mao.classification[0].label

            todas_maos.append(info_mao)
            # Convert to cv::UMat
            img = img.copy()
            mp_desenho.draw_landmarks(img, marcacoes_maos, mp_maos.HAND_CONNECTIONS)

    return img, todas_maos


def dedos_levantados(mao):
    dedos = []
    for ponta_dedo in [8, 12, 16, 20]:
        if mao['coordenadas'][ponta_dedo][1] < mao['coordenadas'][ponta_dedo - 2][1]:
            dedos.append(True)
        else:
            dedos.append(False)
    return dedos

def image_callback(msg):
    global current_image
    current_image = msg

def desenhar_no_quadro(req):
    global img_quadro, cor_pincel, espessura_pincel, x_quadro, y_quadro, desenhando

    if current_image is None:
        return TriggerResponse(success=False, message="Nenhuma imagem recebida")

    # Convertendo a mensagem de imagem para uma estrutura manipulável
    img = bridge.imgmsg_to_cv2(current_image, desired_encoding="passthrough")

    # Detectando mãos na imagem e desenhando landmarks
    img, todas_maos = encontra_coordenadas_maos(img)
    
    # Verificando se há dedos levantados
    dedos_estao_levantados = any(dedos_levantados(mao) for mao in todas_maos)

    if dedos_estao_levantados:
        desenhando = True
        if len(todas_maos) == 1:
            info_dedos_mao1 = dedos_levantados(todas_maos[0])

            if todas_maos[0]['lado'] == 'Right' and info_dedos_mao1 == [True, False, False, True]:
                desenhando = False
        elif len(todas_maos) == 2:
            info_dedos_mao1 = dedos_levantados(todas_maos[0])
            info_dedos_mao2 = dedos_levantados(todas_maos[1])

            indicador_x, indicador_y, indicador_z = todas_maos[0]['coordenadas'][8]

            # Determinando a cor do pincel com base na mão secundária
            if sum(info_dedos_mao2) == 1:
                cor_pincel = AZUL
            elif sum(info_dedos_mao2) == 2:
                cor_pincel = VERDE
            elif sum(info_dedos_mao2) == 3:
                cor_pincel = VERMELHO
            elif sum(info_dedos_mao2) == 4:
                cor_pincel = BRANCO
            else:
                img_quadro = np.ones((resolucao_y, resolucao_x, 3), np.uint8) * 255

            espessura_pincel = int(abs(indicador_z)) // 3 + 5
            cv2.circle(img, (indicador_x, indicador_y), espessura_pincel, cor_pincel, cv2.FILLED)

            if info_dedos_mao1 == [True, False, False, False]:
                if x_quadro == 0 and y_quadro == 0:
                    x_quadro, y_quadro = indicador_x, indicador_y

                cv2.line(img_quadro, (x_quadro, y_quadro), (indicador_x, indicador_y), cor_pincel, espessura_pincel)

                x_quadro, y_quadro = indicador_x, indicador_y
            else:
                x_quadro, y_quadro = 0, 0

            img = cv2.addWeighted(img, 1, img_quadro, 0.2, 0)

    else:
        desenhando = False

    # Exibindo a imagem resultante
    cv2.imshow("Imagem", img)
    cv2.waitKey(1)

    return TriggerResponse(success=True, message="Desenho no quadro atualizado")

def quadro_server():
    rospy.init_node('drawing_service')
    rospy.Subscriber('/Imagens', Image, image_callback)
    rospy.Service('/draw_on_board', Trigger, desenhar_no_quadro)
    rospy.set_param('/stop_counting', False)
    rospy.loginfo("Serviço de desenho no quadro iniciado.")
    rospy.spin()

if __name__ == '__main__':
    try:
        quadro_server()
    except rospy.ROSInterruptException:
        pass