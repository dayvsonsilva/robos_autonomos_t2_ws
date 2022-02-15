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


class ProcessData():
    def __initi__(self):
        self.variavel_a = 0

    # monta nuvem de pontos, converte lar polar para retangula(x,y)
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

        textepose, testeorie = robot.get_odom()
        rospy.loginfo("textepose: " + str(textepose))
        rospy.loginfo("testeorie: " + str(degrees(testeorie)))
        rospy.loginfo("")

        nuvem = [self.points[:, 0], self.points[:, 1]]
        matriz = [self.points[:, 0], self.points[:, 1],
                  np.ones(np.size(nuvem, 1))]

        mapa_alinhado = align_map(matriz, self.x, self.y, radians(self.ow))

        return mapa_alinhado

    def calc_area(self, point_cloud):
        rospy.loginfo("Shape:" + str(np.shape(point_cloud)))
        hull = ConvexHull(point_cloud)
        rospy.loginfo("Area calculada:" + str(hull.area))


# translação
def translacao(tx, ty):
    matriz_de_translacao = np.array([[1.0, 0.0, tx],
                                     [0.0, 1.0, ty],
                                     [0.0, 0.0, 1.0]])
    return matriz_de_translacao

# Rotação


def rotation(angle):
    angle = angle  # * pi/180
    rotation_matrix = np.array([[cos(angle), -sin(angle), 0.0],
                                [sin(angle), cos(angle), 0.0],
                                [0.0, 0.0, 1.0]])
    return rotation_matrix

# Alinha mapa


# def align_map(points, x, y, psir):
#     t1 = translacao(x, y)
#     r1 = rotation(psir)

#     m = np.dot(t1, r1)

#     point_cloud = np.dot(m, points)
#     return point_cloud


def align_map(points, x, y, psir):
    t1 = translacao(x, y)
    r1 = rotation(psir)

    point_cloud_parc = np.dot(r1, points)
    point_cloud = np.dot(t1, point_cloud_parc)

    return point_cloud


robotpd = ProcessData()

color = ['b', 'g', 'r', 'c', 'm', 'y', 'k', 'w']
laser = robot.get_laser()
# (pose, orient) = robot.get_odom()
# xp, yp, orient = robot.get_pose()
# nuvem = robotpd.point_cloud(laser, xp, yp, orient)

# rospy.loginfo("orientação inicial" + str(orient))
rospy.loginfo("Inicio da exploração")

nuvem_final = np.zeros([3, 721])

for i in range(0, 3, 1):
    # Captura laser n i
    laser = robot.get_laser()
    xp, yp, orient = robot.get_pose()  # x(m), y(m), w(degrees)
    nuvem = robotpd.point_cloud(laser, xp, yp, orient)
    # if i % 2 != 0:
    nuvem_final = np.concatenate([nuvem, nuvem_final], axis=1)
    plt.axis('equal')
    plt.scatter(nuvem[0, :], nuvem[1, :], c=color[i])

    # Movimenta Linear
    # laser = robot.get_laser()
    # while ([laser.ranges[330:390]] > (np.ones([1, 60]))*1.3).all():  # 44
    #     # rospy.loginfo("Seguindo em frente...")
    #     # robot.linar_moviment(0.5)
    #     robot.move_straight(0.1)
    #     laser = robot.get_laser()
    #     # time.sleep(1)

    # laser = robot.get_laser()
    # while ([laser.ranges[330:390]] < (np.ones([1, 60]))*1.3).all():  # 44
    #     # rospy.loginfo("Seguindo em ré...")
    #     # robot.linar_moviment(0.5)
    #     robot.move_straight(-0.1)
    #     laser = robot.get_laser()
    #     # time.sleep(1)

    # robot.stop_robot()

    # Rotação
    laser = robot.get_laser()
    if laser.ranges[119] < laser.ranges[599]:
        rospy.loginfo("Virando a esquerda... ")
        robot.rotate(90)
        # time.sleep(1)
    elif laser.ranges[599] < laser.ranges[119]:
        rospy.loginfo("Virando a direita... ")
        robot.rotate(-90)
        # time.sleep(1)

    # # Rotação
    # if laser.ranges[119] > laser.ranges[599]:
    #     robot.rotate(-90)
    #     time.sleep(1)
    # if laser.ranges[599] > laser.ranges[119]:
    #     robot.rotate(90)
    #     time.sleep(1)
    # # robot.rotate(90)
    # time.sleep(2)

    # laser = robot.get_laser()
    # while laser.ranges[360] > 1:
    #     robot.linar_moviment(0.2)
    #     laser = robot.get_laser()
    # time.sleep(1)
    # while laser.ranges[719] > 1:
    #     robot.angular_moviment(0.1)
    #     laser = robot.get_laser()
    # time.sleep(1)
    # robot.rotate(90)
    # robot.stop_robot()

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
