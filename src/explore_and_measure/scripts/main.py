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


rospy.loginfo("Start process...")


class ProcessData():
    def __init__(self):
        self.variavel_a = 0
    # monta nuvem de pontos, converte de polar para retangula(x,y)

    def laser_pol_to_rect(self, laser_data):
        # prepara dados do laser para conversão
        msg = laser_data
        self.laser = msg.ranges
        self.angle_step = degrees(msg.angle_increment) / 57.2958
        self.angle_start = degrees(msg.angle_min)
        self.angle_stop = degrees(msg.angle_max)
        self.points = np.zeros([1, 2])
        angulo = self.angle_start
        # convert laser em um array contento pontos (x, y)
        for i in range(len(self.laser)):
            self.point = np.array([[(cmath.rect(self.laser[i], angulo).real), (cmath.rect(
                self.laser[i], angulo).imag)]])  # x e y
            self.polar_point = np.array(
                [(self.laser[i], angulo), (self.laser[i], angulo)])
            angulo = angulo + self.angle_step  # self.angle_step
            self.points = np.concatenate([self.point, self.points])

        return self.points

    def point_cloud(self, points, x, y, orient):
        self.points = points
        self.x = x
        self.y = y
        self.ow = orient
        nuvem = [self.points[:, 0], self.points[:, 1]]
        matriz = [self.points[:, 0], self.points[:, 1],
                  np.ones(np.size(nuvem, 1))]
        mapa_alinhado = self.align_in_map(matriz, self.x, self.y, self.ow)

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
        m1 = self.np.dot(t1, r1)
        p1 = self.np.dot(m1, points)

        t2 = self.translacao(-x, -y)
        r2 = self.rotation(0)
        m2 = self.np.dot(t2, r2)
        p2 = self.np.dot(m2, p1)

        t3 = self.translacao(-x, -y)
        r3 = self.rotation(0)
        m3 = self.np.dot(t3, r3)
        p3 = self.np.dot(m3, points)

        t4 = self.translacao(-x, -y)
        r4 = self.rotation(0)
        m4 = self.np.dot(t4, r4)
        p4 = self.np.dot(m4, points)

        return p2  # point_cloud

    def change_world2cam(M, point_world):
        M_inv = np.linalg.inv(M)
        p_cam = np.dot(M_inv, point_world)
        return p_cam


robot = RobotControl()
robotpd = ProcessData()


color = ['b', 'g', 'r', 'c', 'm', 'y', 'k', 'w']
_, _, _ = robot.get_pose()
laser_data = robot.get_laser()
# laser_data = robot.get_laser()odometry_msg
rospy.loginfo("Inicio da exploração")

nuvem_final = np.zeros([3, 721])
x_inicial, y_inicial, orient_inicial = robot.get_pose()  # x(m), y(m), w(degrees)

for i in range(0, 2, 1):
    # Captura laser n i
    laser_data = robot.get_laser()  # Lê laser
    xp, yp, orient = robot.get_pose()  # Lẽ pose atual x(m), y(m), w(degrees)

    nuvem_ind = robotpd.laser_pol_to_rect(laser_data)

    nuvem = robotpd.point_cloud(laser_data, xp, yp,  orient_inicial)

    # nuvem = robotpd.point_cloud(laser, xp - x_inicial, yp - y_inicial,  orient_inicial-orient_inicial)

    # x_inicial, y_inicial, orient_inicial = xp, yp, orient
    # if i % 2 != 0:
    nuvem_final = np.concatenate([nuvem, nuvem_final], axis=1)
    plt.axis('equal')
    plt.scatter(nuvem[0, :], nuvem[1, :], c=color[i])

    # Movimenta Linear
    # laser = robot.get_laser()
    # while ([laser.ranges[330:390]] > (np.ones([1, 60]))*1).all():  # 44
    #     # rospy.loginfo("Seguindo em frente...")
    #     # robot.linar_moviment(0.5)
    #     robot.move_straight(0.2)
    #     laser = robot.get_laser()
    #     # time.sleep(1)

    # laser = robot.get_laser()
    # while ([laser.ranges[330:390]] < (np.ones([1, 60]))*1.5).all():  # 44
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

    # # Rotação
    if laser_data.ranges[119] > laser_data.ranges[599]:
        robot.rotate(-90)
        time.sleep(1)
    if laser_data.ranges[599] > laser_data.ranges[119]:
        robot.rotate(90)
        time.sleep(1)
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
