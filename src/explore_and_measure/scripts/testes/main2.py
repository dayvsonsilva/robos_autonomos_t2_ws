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


class ProcessData:
    def __init__(self):
        self.variavel_a = 0
        self.points = np.zeros([1, 2])
    # monta nuvem de pontos, converte de polar para retangula(x,y)

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

        mapa_alinhado = self.align_in_map(
            matriz, self.x, self.y, radians(self.ow))

        return mapa_alinhado

    def calc_area(self, point_cloud):
        rospy.loginfo("Shape:" + str(np.shape(point_cloud)))
        hull = ConvexHull(point_cloud)
        rospy.loginfo("Area calculada:" + str(hull.area))

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
        t1 = self.translacao(x, y)
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

        return p1 # point_cloud

    def change_world2cam(M, point_world):
        M_inv = np.linalg.inv(M)
        p_cam = np.dot(M_inv, point_world)
        return p_cam


robotpd = ProcessData()


color = ['b', 'g', 'r', 'c', 'm', 'y', 'k', 'w']
_, _, _ = robot.get_pose()
laser_data = robot.get_laser()
# laser_data = robot.get_laser()
rospy.loginfo("Inicio da exploração")

nuvem_final = np.zeros([3, 721])
x_anterior, y_anterior, orient_anterior = robot.get_pose()  # x(m), y(m), w(degrees)


for i in range(0, 2, 1):
    laser = robot.get_laser()  # Verifico laser
    xp, yp, orient = robot.get_pose()  # verifico pose x(m), y(m), w(degrees)
    # converto em nuvem de pontos
    nuvem = robotpd.point_cloud(
        laser, x_anterior-xp, x_anterior-yp, orient_anterior-orient)

    x_anterior, y_anterior, orient_anterior = xp, yp, orient

    # if i % 2 != 0:
    nuvem_final = np.concatenate([nuvem, nuvem_final], axis=1)
    plt.axis('equal')
    plt.scatter(nuvem[0, :], nuvem[1, :], c=color[i])
    # Captura laser n i
    # laser_data = robot.get_laser()  # Lê laser
    # xp, yp, orient = robot.get_pose()  # Lẽ pose atual x(m), y(m), w(degrees)

    # nuvem_ind = robotpd.laser_pol_to_rect(laser_data)

    # # nuvem = robotpd.point_cloud(laser_data, xp, yp,  orient)
    # rospy.loginfo("antes da função point cloud")
    # nuvem = robotpd.point_cloud(
    #     nuvem_ind, xp - x_inicial, yp - y_inicial,  orient_inicial-orient_inicial)
    # # nuvem = robotpd.point_cloud(
    # #     xp - x_inicial, yp - y_inicial,  orient_inicial-orient_inicial)
    # rospy.loginfo("depois da função")

    # # x_inicial, y_inicial, orient_inicial = xp, yp, orient
    # # if i % 2 != 0:
    # nuvem_final = np.concatenate([nuvem, nuvem_final], axis=1)
    # plt.axis('equal')
    # plt.scatter(nuvem[0, :], nuvem[1, :], c=color[i])

    # Movimenta Linear
    laser = robot.get_laser()
    while ([laser.ranges[330:390]] > (np.ones([1, 60]))*1).all():  # 44
        # rospy.loginfo("Seguindo em frente...")
        # robot.linar_moviment(0.5)
        robot.move_straight(0.2)
        laser = robot.get_laser()
        # time.sleep(1)
    robot.stop_robot()

    # laser = robot.get_laser()
    # while ([laser.ranges[330:390]] < (np.ones([1, 60]))*1.2).all():  # 44
    #     # rospy.loginfo("Seguindo em ré...")
    #     # robot.linar_moviment(0.5)
    #     robot.move_straight(-0.2)
    #     laser = robot.get_laser()
    #     # time.sleep(1)

    # robot.stop_robot()

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

    # # # Rotação
    # if laser_data.ranges[119] > laser_data.ranges[599]:
    #     robot.rotate(-90)
    #     time.sleep(1)
    # elif laser_data.ranges[599] > laser_data.ranges[119]:
    #     robot.rotate(90)
    #     time.sleep(1)
    # robot.rotate(90)
    time.sleep(2)

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
