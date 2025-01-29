import rclpy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from message_filters import Subscriber, ApproximateTimeSynchronizer
from rclpy.node import Node
from std_msgs.msg import Int16
import math
from sklearn.cluster import DBSCAN
import matplotlib.pyplot as plt
import numpy as np


class Object_classifier(Node):

    def __init__(self):

        super().__init__('Object_classifier') #Iniciando classe base "Node"
        
        #Subscribers
        scan_data = Subscriber(self, LaserScan, '/scan')
        odometry = Subscriber(self, Odometry, '/odom')

        #Sync msgs
        self.ts = ApproximateTimeSynchronizer([scan_data, odometry], 10, 0.1)
        self.ts.registerCallback(self.scan)

        #Publishers
        self.publisher = self.create_publisher(Int16, "/clusters", 10)
        self.pub_msg = Int16()

        #Initial states
        self.scan_init = False
        self.scan_points = None

        #Plot
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [], 'bo', markersize=2)
        self.line_robot, = self.ax.plot([], [], 'k', markersize=2)



    def scan(self, scan_data, odometry):
        if not self.scan_init:
            self.scan_init = True

        self.convert_coordinates(scan_data, odometry)

        self.update_plot()



    #def update_plot(self):
    #    """
    #    Atualiza o gráfico com os pontos do scan e a posição do robô.
    #    """
    #    x_data = self.scan_points[:, 0]
    #    y_data = self.scan_points[:, 1]
    #    self.line.set_data(x_data, y_data)
    #    self.line_robot.set_data(self.pose_odom[0], self.pose_odom[1])
    #    self.ax.relim()
    #    self.ax.autoscale_view()
    #    plt.pause(0.01)

    def update_plot(self):
        """
        Atualiza o gráfico com os pontos do scan e a posição do robô.
        """
        if self.scan_points is None or len(self.scan_points) == 0:
            return

        # Aplicar DBSCAN para obter clusters
        db = DBSCAN(eps=0.15, min_samples=5).fit(self.scan_points)
        cluster_labels = db.labels_

        # Criar um mapa de cores
        unique_labels = set(cluster_labels)
        colors = plt.cm.jet(np.linspace(0, 1, len(unique_labels)))  # Gera cores variadas

        self.ax.clear()  # Limpa o gráfico para redesenhar
        
        for label, color in zip(unique_labels, colors):
            cluster_points = self.scan_points[cluster_labels == label]
            if label == -1:  # Pontos considerados ruído (parede)
                color = 'black'
                marker = 's'  # Quadrado para indicar parede
                label_text = "Parede"
            else:
                color = color  # Cor dinâmica do cluster
                marker = 'o'  # Círculo para clusters
                label_text = f"Cluster {label} ({np.mean(cluster_points[:, 0]):.2f}, {np.mean(cluster_points[:, 1]):.2f})"

            self.ax.scatter(cluster_points[:, 0], cluster_points[:, 1], c=[color], marker=marker, s=5, label=label_text)
        
        # Adicionar a posição do robô
        self.ax.scatter(self.pose_odom[0], self.pose_odom[1], c='k', marker='x', s=100, label="Robô")
        
        self.ax.legend()
        self.ax.relim()
        self.ax.autoscale_view()
        plt.pause(0.01)



    def convert_coordinates(self, scan_data, odometry):
        """
        Converte coordenadas do LiDAR para o referencial global utilizando a odometria.
        """
        _, _, yaw = convert_quaternion(odometry.pose.pose.orientation)
        self.pose_odom = np.array([odometry.pose.pose.position.x, odometry.pose.pose.position.y])

        for index in range(len(scan_data.ranges)):
            if scan_data.range_min < scan_data.ranges[index] < scan_data.range_max:
                angle = index * scan_data.angle_increment
                P_S = np.array([[np.cos(angle) * scan_data.ranges[index]], # Matriz de transformação do sensor para o LiDAR
                                [np.sin(angle) * scan_data.ranges[index]],
                                [1]])
                M_PL = np.array([[1, 0, -0.064], # Matriz de transformação do LiDAR para o mundo
                                    [0, 1, 0],
                                    [0, 0, 1]])
                M_LW = np.array([[np.cos(yaw), -np.sin(yaw), self.pose_odom[0]],  # Ponto no referencial do sensor (P_S)
                                    [np.sin(yaw), np.cos(yaw), self.pose_odom[1]],
                                    [0, 0, 1]])
                pose_tf = M_LW @ M_PL @ P_S  # Transforma o ponto para o referencial global
                self.scan_points = sub_sampling(pose_tf.T[0][0:2], self.scan_points)

        cluster_len = classifier(self.scan_points)
        self.pub_msg.data = cluster_len
        self.publisher.publish(self.pub_msg)


def circle_fitting(cluster_points):

    x = cluster_points[:,0]
    y = cluster_points[:,1]

    sumx = sum(x)
    sumy = sum(y)
    sumx2 = sum([ix ** 2 for ix in x])
    sumy2 = sum([iy ** 2 for iy in y])
    sumxy = sum([ix * iy for (ix, iy) in zip(x, y)])

    F = np.array([[sumx2, sumxy, sumx],
                  [sumxy, sumy2, sumy],
                  [sumx, sumy, len(x)]])
    G = np.array([[-sum([ix ** 3 + ix * iy ** 2 for (ix, iy) in zip(x, y)])],
                  [-sum([ix ** 2 * iy + iy ** 3 for (ix, iy) in zip(x, y)])],
                  [-sum([ix ** 2 + iy ** 2 for (ix, iy) in zip(x, y)])]])
    T = np.linalg.inv(F).dot(G)

    cx = float(T[0, 0] / -2) # Extração das coordenadas do centro e do raio
    cy = float(T[1, 0] / -2)

    raio = math.sqrt(cx**2 + cy**2 - T[2, 0])

    # Calculando todas as distâncias dos pontos ao centro do círculo
    distances = [np.hypot(cx - ix, cy - iy) for (ix, iy) in zip(x, y)]

    max_dist = max(distances) # Maior distância

    error = sum([(dist / max_dist) - raio/max_dist for dist in distances])

    return (cx, cy, raio, error)


def get_circle_diameter(radius):
    """
    Calcula o diâmetro de um círculo dado o raio.
    """
    return 2 * radius


def get_rectangle_dimensions(cluster_points):
    """
    Estima a largura e altura de um retângulo ajustando um bounding box nos pontos do cluster.
    """
    x_min, y_min = np.min(cluster_points, axis=0)
    x_max, y_max = np.max(cluster_points, axis=0)
    width = x_max - x_min
    height = y_max - y_min
    return width, height


def classifier(scan_points):
    cluster_len = 0
    db = DBSCAN(eps=0.15, min_samples=5).fit(scan_points)
    cluster_labels = db.labels_
    print("\n\n-----------------------------------------\n")

    for label in set(cluster_labels):
        if label == -1:
            continue
        cluster_points = scan_points[cluster_labels == label]
        if cluster_points.shape[0] > cluster_len:
            cluster_len = cluster_points.shape[0]
        try:
            center_x, center_y, raio, e = circle_fitting(cluster_points)
            if raio < 2:
                if np.abs(e) <= 0.15:
                    diameter = get_circle_diameter(raio)
                    print(f"Círculo (fit: {abs(e):.3f}) - Posição: X: {center_x:.1f}, Y: {center_y:.1f}, Diâmetro: {diameter:.2f}")
                else:
                    width, height = get_rectangle_dimensions(cluster_points)
                    print(f"Retângulo (fit: {abs(e):.3f}) - Posição: X: {center_x:.1f}, Y: {center_y:.1f}, Largura: {width:.2f}, Altura: {height:.2f}")
        except:
            pass

    return cluster_len



def convert_quaternion(quaternion):
    x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w

    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z

    roll = np.arctan2(2 * (wx + yz), 1 - 2 * (xx + yy))
    pitch = np.arcsin(2 * (wy - xz))
    yaw = np.arctan2(2 * (wz + xy), 1 - 2 * (yy + zz))

    return roll, pitch, yaw



def sub_sampling(pose_tf, array_pts):

    pose_tf = np.array([pose_tf])
    if array_pts is None:
        array_pts = pose_tf
    else:
        pose_repeat = np.repeat(pose_tf, repeats = [array_pts.shape[0]], axis=0)
        dist = np.linalg.norm(array_pts - pose_repeat, axis = 1)
        if np.any(dist <= 0.05):
            pass
        else:
            array_pts = np.vstack((array_pts,pose_tf))

    return array_pts


def main():

    rclpy.init()
    node = Object_classifier()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()