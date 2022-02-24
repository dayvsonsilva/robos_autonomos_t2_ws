#! /usr/bin/env python3
import rospy
from math import pi, cos, sin, sqrt, atan, degrees, radians
import cmath
import numpy as np
import time
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from robot_control import RobotControl
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull, convex_hull_plot_2d


robot = RobotControl()
rospy.loginfo("Start process...")

###########################################################
# Classe deprocessamento de dados
#
#


class ProcessData:
    def __init__(self):
        self.variavel_a = 0
        self.points = np.zeros([1, 2])

    ###########################################################
    # monta nuvem de pontos, converte de polar para retangula(x,y)
    # Entradas : laser - mensagem recebida do laser
    #            x, y, orient - pose do robo no momento da
    #                           leitura do laser
    #
    # Saida: Nuvem de ponto referenciados a origem(0,0) do mapa
    #

    def point_cloud(self, laser, x, y, orient):
        msg = laser
        self.laser = msg.ranges
        self.angle_step = degrees(msg.angle_increment) / 57.2958
        self.angle_start = degrees(msg.angle_min)
        self.angle_stop = degrees(msg.angle_max)
        # rospy.loginfo("Laser len:" + str(len(self.laser)))
        self.points = np.zeros([1, 2])
        angulo = self.angle_start
        # convert laser em um array
        for i in range(len(self.laser)):
            self.point = np.array([[(cmath.rect(self.laser[i], angulo).real), (cmath.rect(
                self.laser[i], angulo).imag)]])  # x e y
            self.polar_point = np.array(
                [(self.laser[i], angulo), (self.laser[i], angulo)])
            angulo = angulo + self.angle_step  # self.angle_step
            self.points = np.concatenate([self.point, self.points])

        self.x = x
        self.y = y
        self.ow = orient
        rospy.loginfo("")
        rospy.loginfo("x: " + str(self.x))
        rospy.loginfo("y: " + str(self.y))
        rospy.loginfo("Angulo na captura: " + str(self.ow))
        rospy.loginfo("")

        # textepose, testeorie = robot.get_odom()
        # rospy.loginfo("textepose: " + str(textepose))
        # rospy.loginfo("testeorie: " + str(degrees(testeorie)))
        # rospy.loginfo("")

        nuvem = [self.points[:, 0], self.points[:, 1]]
        matriz = [self.points[:, 0], self.points[:, 1],
                  np.ones(np.size(nuvem, 1))]

        mapa_alinhado = self.align_in_map(
            matriz, self.x, self.y, radians(self.ow))

        return mapa_alinhado

    ###########################################################
    # Calcula area do ambiente a partir da nuvem de pontos
    #
    #

    def calc_area(self, point_cloud):
        rospy.loginfo("Shape:" + str(np.shape(point_cloud)))
        hull = ConvexHull(point_cloud, qhull_options='qt')

        plt.plot(point_cloud[:, 0], point_cloud[:, 1], 'o')
        for simplex in hull.simplices:
            plt.plot(point_cloud[simplex, 0], point_cloud[simplex, 1], 'k-')

        plt.plot(point_cloud[hull.vertices, 0],
                 point_cloud[hull.vertices, 1], 'r--', lw=2)

        plt.plot(point_cloud[hull.vertices[0], 0],
                 point_cloud[hull.vertices[0], 1], 'ro')
        plt.axis('equal')
        plt.show()

        rospy.loginfo("Area calculada: " + str(round(hull.area, 2)))

    ###########################################################
    # Funções de translação e rotação
    #
    #

    # translação

    def translacao(self, tx, ty):
        matriz_de_translacao = np.array([[1.0, 0.0, tx],
                                         [0.0, 1.0, ty],
                                         [0.0, 0.0, 1.0]])
        return matriz_de_translacao

    # Rotação
    def rotation(self, angle):
        angle = angle  # * pi/180
        rotation_matrix = np.array([[cos(angle), -sin(angle), 0.0],
                                    [sin(angle), cos(angle), 0.0],
                                    [0.0, 0.0, 1.0]])
        return rotation_matrix

    def align_in_map(self, points, x, y, psir):
        # angulo_offset = 134.99974079476908173
        # mudar referencial do robô para o mundo
        t1 = self.translacao(-x, -y)
        r1 = self.rotation(psir)
        m1 = np.dot(r1, t1)
        m1_inv = np.linalg.inv(m1)
        p1 = np.dot(m1_inv, points)

        t2 = self.translacao(-x, -y)
        r2 = self.rotation(0)
        m2 = np.dot(t2, r2)
        p2 = np.dot(m2, p1)

        t3 = self.translacao(-x, -y)
        r3 = self.rotation(0)
        m3 = np.dot(t3, r3)
        p3 = np.dot(m3, points)

        t4 = self.translacao(-x, -y)
        r4 = self.rotation(0)
        m4 = np.dot(t4, r4)
        p4 = np.dot(m4, points)

        return p1  # point_cloud

    def change_world2cam(M, point_world):
        M_inv = np.linalg.inv(M)
        p_cam = np.dot(M_inv, point_world)
        return p_cam


if __name__ == "__main__":

    robotpd = ProcessData()  # Intancia objeto processdata

    color = ['b', 'g', 'r', 'c', 'm', 'y', 'k', 'w']
    _, _, _ = robot.get_pose()

    laser_data = robot.get_laser()
    rospy.loginfo("Inicio da exploração")

    nuvem_final = np.zeros([3, 721])
    # Realiza primeira captura da nuvem de pontos
    x_anterior, y_anterior, orient_anterior = robot.get_pose()  # x(m), y(m), w(degrees)

    ###########################################################
    # Loop contendo função de captura de dados do laser, alinhamento
    # e movimentação do robô
    #

    for i in range(0, 2, 1):
        ###########################################################
        # Captura e alinhamento da imagem
        #
        #

        laser = robot.get_laser()  # Verifico laser
        xp, yp, orient = robot.get_pose()  # verifico pose x(m), y(m), w(degrees)

        ##########################################################################################
        tf_data = robot.get_tf()
        # trans = tf_data.lookup_transform('odom', 'base_link', rospy.Time())

        ##########################################################################################

        # converto laser em nuvem de pontos
        nuvem = robotpd.point_cloud(
            laser,  x_anterior-xp, y_anterior-yp, orient_anterior-orient)

        nuvem_final = np.concatenate([nuvem, nuvem_final], axis=1)
        plt.axis('equal')
        plt.scatter(nuvem[0, :], nuvem[1, :], c=color[i])
        plt.show()

        ###########################################################
        # Movimentação do robô
        #
        #
        # Rotação
        laser_data = robot.get_laser()
        if laser_data.ranges[119] < laser_data.ranges[599]:
            rospy.loginfo("Virando a esquerda... ")
            robot.rotate(90)
            # time.sleep(1)
        elif laser_data.ranges[599] < laser_data.ranges[119]:
            rospy.loginfo("Virando a direita... ")
            robot.rotate(-90)
        time.sleep(1)
        robot.stop_robot()

    plt.axis('equal')
    plt.show()
    robotpd.calc_area(np.transpose([nuvem_final[0, :], nuvem_final[1, :]]))

    plt.show()
    plt.axis('equal')
    plt.scatter(nuvem_final[0, :], nuvem_final[1, :], c=color[6])
    plt.show()

    # PENDENCIAS###################################################
    # converter nuvem_final em polar(raio e theta)
    # Criar função para gerar triangulos entre o ponto (0,0) e as extremidades do laser
    # Calcular a soma de todos os triangulos gerados
    # apresentar como ros info area em m²
    # PENDENCIAS###################################################

    # # rospy.loginfo("Main callback")
    rospy.loginfo("END")


#########################################################
# Exemplos de acionamentos
#
#

# Rotação
# laser_data = robot.get_laser()
# if laser_data.ranges[119] < laser_data.ranges[599]:
#     rospy.loginfo("Virando a esquerda... ")
#     robot.rotate(90)
#     # time.sleep(1)
# elif laser_data.ranges[599] < laser_data.ranges[119]:
#     rospy.loginfo("Virando a direita... ")
#     robot.rotate(-90)
# time.sleep(1)

# # Movimenta Linear
# laser = robot.get_laser()
# while ([laser.ranges[330:390]] > (np.ones([1, 60]))*1).all():  # 44
#     # rospy.loginfo("Seguindo em frente...")
#     # robot.linar_moviment(0.5)
#     robot.move_straight(0.2)
#     laser = robot.get_laser()
#     # time.sleep(1)
# robot.stop_robot()

# laser = robot.get_laser()
# while ([laser.ranges[330:390]] < (np.ones([1, 60]))*1.2).all():  # 44
#     # rospy.loginfo("Seguindo em ré...")
#     # robot.linar_moviment(0.5)
#     robot.move_straight(-0.2)
#     laser = robot.get_laser()
#     # time.sleep(1)

# robot.stop_robot()
