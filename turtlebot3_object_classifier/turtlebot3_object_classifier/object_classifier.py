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
import sys


class Object_classifier(Node):

    def __init__(self):

        super().__init__("Object_classifier")  # Iniciando classe base "Node"

        # Subscribers
        scan_data = Subscriber(self, LaserScan, "/scan")
        odometry = Subscriber(self, Odometry, "/odom")

        # Sync msgs
        self.ts = ApproximateTimeSynchronizer([scan_data, odometry], 10, 0.1)
        self.ts.registerCallback(self.scan)

        # Publishers
        self.publisher = self.create_publisher(Int16, "/clusters", 10)
        self.pub_msg = Int16()

        # Initial states
        self.scan_init = False
        self.scan_points = None

        # Plot
        self.fig, self.ax = plt.subplots()
        (self.line,) = self.ax.plot([], [], "bo", markersize=2)
        (self.line_robot,) = self.ax.plot([], [], "k", markersize=2)

        self.previous_max_points = 0  # Variável para armazenar o número de pontos do maior cluster na iteração anterior
        self.no_growth_counter = 0
        self.NO_GROWTH_LIMIT = 12
        self.classified_clusters = {}
        self.classification_info = []

    def scan(self, scan_data, odometry):
        if not self.scan_init:
            self.scan_init = True

        self.convert_coordinates(scan_data, odometry)

    def convert_coordinates(self, scan_data, odometry):
        """
        Converte coordenadas do LiDAR para o referencial global utilizando a odometria.
        """
        _, _, yaw = convert_quaternion(odometry.pose.pose.orientation)
        self.pose_odom = np.array([odometry.pose.pose.position.x, odometry.pose.pose.position.y])

        for index in range(len(scan_data.ranges)):
            if scan_data.range_min < scan_data.ranges[index] < scan_data.range_max:
                angle = index * scan_data.angle_increment
                P_S = np.array(
                    [
                        [np.cos(angle) * scan_data.ranges[index]],  # Matriz de transformação do sensor para o LiDAR
                        [np.sin(angle) * scan_data.ranges[index]],
                        [1],
                    ]
                )
                M_PL = np.array([[1, 0, -0.064], [0, 1, 0], [0, 0, 1]])  # Matriz de transformação do LiDAR para o mundo
                M_LW = np.array(
                    [
                        [np.cos(yaw), -np.sin(yaw), self.pose_odom[0]],  # Ponto no referencial do sensor (P_S)
                        [np.sin(yaw), np.cos(yaw), self.pose_odom[1]],
                        [0, 0, 1],
                    ]
                )
                pose_tf = M_LW @ M_PL @ P_S  # Transforma o ponto para o referencial global
                self.scan_points = sub_sampling(pose_tf.T[0][0:2], self.scan_points)

        cluster_len = classifier(self.scan_points, self)
        self.pub_msg.data = cluster_len
        self.publisher.publish(self.pub_msg)


def update_plot(scan_points, cluster_labels, classified_clusters, self):
    """
    Atualiza o gráfico com os pontos do scan, destacando os clusters com cores diferentes.
    """
    if scan_points is None or len(scan_points) == 0:
        return

    unique_labels = set(cluster_labels)
    colors = plt.cm.jet(np.linspace(0, 1, len(unique_labels)))  # Gera cores variadas

    self.ax.clear()  # Limpa o gráfico atual

    max_points = 0
    max_label = None

    for label, color in zip(unique_labels, colors):
        if label == -1:
            continue  # Ignorar o ruído
        cluster_points = scan_points[cluster_labels == label]

        # Verificar se este cluster tem mais pontos
        if len(cluster_points) > max_points:
            max_points = len(cluster_points)
            max_label = label

        # Plotando os pontos do cluster com a cor correspondente
        scatter = self.ax.scatter(
            cluster_points[:, 0], cluster_points[:, 1], c=[color], marker="o", s=10, label=f"Cluster {label}"
        )

        # Se o cluster for classificado, incluir informações adicionais na legenda
        if label in classified_clusters:
            cluster_info = classified_clusters[label]
            # Referência à posição, tipo e tamanho na legenda
            shape_type = cluster_info["type"]

            # Atualizando o rótulo da legenda com as informações adicionais
            scatter.set_label(f"Cluster {label} - {shape_type.capitalize()}")

    # Rotular o maior objeto como 'paredes externas'
    if max_label is not None:
        cluster_points = scan_points[cluster_labels == max_label]
        self.ax.scatter(
            cluster_points[:, 0],
            cluster_points[:, 1],
            c="red",
            marker="o",
            s=5,
            label="paredes externas",
            edgecolors="r",
            linewidths=2,
        )

    # Adicionando legenda

    # box = self.ax.get_position()
    # self.ax.set_position([box.x0, box.y0, box.width * 0.8, box.height])

    # self.ax.legend(title="Clusters", loc="upper left", bbox_to_anchor=(1, 0.5), fontsize=10)

    # Títulos e rótulos
    self.ax.set_title("Visualização dos Clusters", fontsize=14)
    self.ax.set_xlabel("Eixo X", fontsize=12)
    self.ax.set_ylabel("Eixo Y", fontsize=12)

    # Atualiza a posição do robô (caso necessário)
    self.line_robot.set_data(self.pose_odom[0], self.pose_odom[1])
    self.ax.relim()
    self.ax.autoscale_view()

    plt.pause(0.01)


def circle_fitting(cluster_points):

    x = cluster_points[:, 0]
    y = cluster_points[:, 1]

    sumx = sum(x)
    sumy = sum(y)
    sumx2 = sum([ix**2 for ix in x])
    sumy2 = sum([iy**2 for iy in y])
    sumxy = sum([ix * iy for (ix, iy) in zip(x, y)])

    F = np.array([[sumx2, sumxy, sumx], [sumxy, sumy2, sumy], [sumx, sumy, len(x)]])
    G = np.array(
        [
            [-sum([ix**3 + ix * iy**2 for (ix, iy) in zip(x, y)])],
            [-sum([ix**2 * iy + iy**3 for (ix, iy) in zip(x, y)])],
            [-sum([ix**2 + iy**2 for (ix, iy) in zip(x, y)])],
        ]
    )
    T = np.linalg.inv(F).dot(G)

    cx = float(T[0, 0] / -2)  # Extração das coordenadas do centro e do raio
    cy = float(T[1, 0] / -2)

    raio = math.sqrt(cx**2 + cy**2 - T[2, 0])

    distances = [np.hypot(cx - ix, cy - iy) for (ix, iy) in zip(x, y)]  # distâncias dos pontos ao centro do círculo

    max_dist = max(distances)  # Maior distância

    error = sum([(dist / max_dist) - raio / max_dist for dist in distances])

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


def classifier(scan_points, self):
    cluster_len = 0
    db = DBSCAN(eps=0.1, min_samples=4).fit(scan_points)
    cluster_labels = db.labels_
    classified_clusters = {}

    print("\n\n-----------------------------------------\n")

    # Identificar o cluster com mais pontos
    max_points = 0
    max_label = None
    for label in set(cluster_labels):
        if label == -1:
            continue
        cluster_points = scan_points[cluster_labels == label]
        if len(cluster_points) > max_points:
            max_points = len(cluster_points)
            max_label = label

    print(f"O maior cluster é o {max_label} com {max_points} pontos, anteriormente existiam {self.previous_max_points}")
    print(f"Restão {self.NO_GROWTH_LIMIT - (self.no_growth_counter)} tentativas para encerrar \n")
    if max_points == self.previous_max_points:
        self.no_growth_counter += 1
        if self.no_growth_counter >= self.NO_GROWTH_LIMIT:
            # Salvar o último print em um arquivo .txt
            with open("ultimo_print.txt", "w") as file:
                for info in self.classification_info:
                    file.write(info + "\n")  # Fechar o código
            # sys.exit()
    else:
        self.no_growth_counter = 0
    self.previous_max_points = max_points

    self.classification_info.clear()

    for label in set(cluster_labels):
        if label == -1 or label == max_label:
            continue
        cluster_points = scan_points[cluster_labels == label]
        if cluster_points.shape[0] > cluster_len:
            cluster_len = cluster_points.shape[0]
        try:

            center_x, center_y, raio, e = circle_fitting(cluster_points)
            if np.abs(e) <= 0.58:
                diameter = get_circle_diameter(raio)
                classified_clusters[label] = {
                    "type": "circle",
                    "center_x": center_x,
                    "center_y": center_y,
                    "diameter": diameter,
                }
                info = f"{label}. Círculo (fit: {abs(e):.3f}) - Posição: X: {center_x:.1f}, Y: {center_y:.1f}, Diâmetro: {diameter:.2f}"
                print(info)
                self.classification_info.append(info)
            else:
                width, height = get_rectangle_dimensions(cluster_points)
                classified_clusters[label] = {
                    "type": "rectangle",
                    "center_x": center_x,
                    "center_y": center_y,
                    "width": width,
                    "height": height,
                }
                info = f"{label}. Retângulo (fit: {abs(e):.3f}) - Posição: X: {center_x:.1f}, Y: {center_y:.1f}, Largura: {width:.2f}, Altura: {height:.2f}"
                print(info)
                self.classification_info.append(info)
        except:
            pass

    update_plot(scan_points, cluster_labels, classified_clusters, self)
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
        pose_repeat = np.repeat(pose_tf, repeats=[array_pts.shape[0]], axis=0)
        dist = np.linalg.norm(array_pts - pose_repeat, axis=1)
        if np.any(dist <= 0.03):
            pass
        else:
            array_pts = np.vstack((array_pts, pose_tf))

    return array_pts


def main():

    rclpy.init()
    node = Object_classifier()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
