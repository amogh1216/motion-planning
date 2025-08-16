#! /usr/bin/python3
# -*- coding: utf-8 -*-

import sys
import os
import math
import numpy as np
import cv2
from pathlib import Path

from PyQt5.Qt import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from path_tracking_simulator_window import Ui_Form

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry

class Map_GraphicsScene(QGraphicsScene):
    def __init__(self, parent=None):
        QGraphicsScene.__init__(self, parent)
        self.center_x = 250
        self.center_y = 250
        self.prev_x = 250
        self.prev_y = 250
        self.scale = 0.02 #m/pix
        self.path = []
        self.path.append([(self.prev_x - self.center_x)*self.scale, 
                     -1*(self.prev_y - self.center_y)*self.scale])
        self.path_pen = QPen(Qt.blue)
        self.obstacle_pen = QPen(Qt.darkYellow)
        self.mode = 'path'  # 'path' or 'obstacle'
        self.path_items = []
        self.obstacle_items = []
        self.drawing_path = False
        self.drawing_obstacle = False

    def set_mode(self, mode):
        self.mode = mode
        if mode == 'path':
            # Clear previous path drawing
            for item in self.path_items:
                try:
                    self.removeItem(item)
                except Exception as e:
                    print(f"Failed to remove item from scene: {e}")
            self.path_items.clear()
            self.path.clear()
            self.prev_x = self.center_x
            self.prev_y = self.center_y
            self.path.append([(self.prev_x - self.center_x)*self.scale, 
                         -1*(self.prev_y - self.center_y)*self.scale])
        # Do not clear obstacles when switching to obstacle mode

    def clear_obstacles(self):
        for item in self.obstacle_items:
            self.removeItem(item)
        self.obstacle_items.clear()

    def mouseMoveEvent(self,event):
        x = event.scenePos().x()
        y = event.scenePos().y()
        if self.mode == 'path':
            line = self.addLine(QLineF(self.prev_x, self.prev_y, x, y), self.path_pen)
            self.path_items.append(line)
            self.path.append([(x - self.center_x)*self.scale, 
                              -1*(y - self.center_y)*self.scale])
            self.drawing_path = True

            self.prev_x = x
            self.prev_y = y 
        elif self.mode == 'obstacle':
            if (len(self.obstacle_items) > 0 and self.drawing_obstacle):
                self.removeItem(self.obstacle_items[-1])
            self.drawing_obstacle = True
            line = self.addLine(QLineF(self.prev_x, self.prev_y, x, y), self.obstacle_pen)
            self.obstacle_items.append(line) 

    def mousePressEvent(self, event):
        x = event.scenePos().x()
        y = event.scenePos().y()
        if self.mode == 'path':
            if self.drawing_path:
                # If already drawing, clear previous path
                for item in self.path_items:
                    try:
                        item.prepareGeometryChange()
                        self.removeItem(item)
                    except Exception as e:
                        print(f"Failed to remove item from scene: {e}")
                self.path_items.clear()
                self.path.clear()
                self.prev_x = x
                self.prev_y = y
                self.path.append([(self.prev_x - self.center_x)*self.scale, 
                             -1*(self.prev_y - self.center_y)*self.scale])
            else:
                self.prev_x = x
                self.prev_y = y
                self.path = [[(self.prev_x - self.center_x)*self.scale, 
                              -1*(self.prev_y - self.center_y)*self.scale]]
            self.drawing_path = False
        elif self.mode == 'obstacle':
            self.drawing_obstacle = False
            self.prev_x = x
            self.prev_y = y

class Path_Tracking_Simulator(QDialog):
    def __init__(self,parent=None):

        super(Path_Tracking_Simulator, self).__init__(parent)
        self.ui = Ui_Form()
        self.ui.setupUi(self)

        self.map_scene = Map_GraphicsScene(self.ui.map_graphicsView)
        self.ui.map_graphicsView.setScene(self.map_scene)

        rclpy.init(args=None)
        self.ros_node = Node('path_tracking_sim')
        self.pub = self.ros_node.create_publisher(Float64MultiArray, '/path', 10)
        self.sub = self.ros_node.create_subscription(TFMessage, '/world/my_world/pose/info', self.listener_callback, 10)
        self.sub_odom = self.ros_node.create_subscription(Odometry, '/rwd_diff_controller/odom', self.odom_callback, 10)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update)
        self.timer.start(10)      

        self.pixmap = QPixmap(500, 500)
        self.vehicle_pen = QPen(Qt.red)
        self.center_line=QPen(Qt.black)
        self.center_line.setStyle(Qt.DashLine)
        self.map_scene.addLine(QLineF(250, 0, 250, 500), self.center_line) 
        self.map_scene.addLine(QLineF(0, 250, 500, 250), self.center_line)

        # odom localization display (green circle)
        self.diameter = 5 #pix
        self.C = 10 #length
        self.first_time = True
        self.enable_repaint = False
        self.odom_x = None
        self.odom_y = None
        self.odom_pen = QPen(Qt.darkGreen)

        self.ui.draw_path_pushButton.clicked.connect(self.enable_draw_path)
        self.ui.draw_obstacle_pushButton.clicked.connect(self.enable_draw_obstacle)
        self.ui.clear_obstacles_pushButton.clicked.connect(self.clear_obstacles)

        self.circle_radius = 50  # in pixels, default value, divided by scale = m
        # Optionally, load from params or config file

    def odom_callback(self, msg):
        # Extract x, y from Odometry message
        x = msg.pose.pose.position.x / self.map_scene.scale + self.map_scene.center_x
        y = -msg.pose.pose.position.y / self.map_scene.scale + self.map_scene.center_y
        self.odom_x = x
        self.odom_y = y

    def listener_callback(self, data):
        for tfsf in data.transforms:
            if(tfsf.child_frame_id == 'rwd_bot'):
                pose = tfsf.transform.translation
                orientation = tfsf.transform.rotation
                x = pose.x/self.map_scene.scale + self.map_scene.center_x
                y = -pose.y/self.map_scene.scale + self.map_scene.center_y
                q0 = orientation.x
                q1 = orientation.y
                q2 = orientation.z
                q3 = orientation.w
                numerator = q0*q1 + q2*q3
                denominator = q0**2 - q1**2 - q2**2 + q3**2
                angle = -math.pi/2 + math.atan2(2*numerator, denominator)

                if(self.enable_repaint):
                    if self.first_time == False:
                        self.map_scene.clear()
                    else:
                        self.first_time = False
                    self.map_scene.addPixmap(self.pixmap)
                    self.map_scene.addLine(QLineF(250, 0, 250, 500), self.center_line) 
                    self.map_scene.addLine(QLineF(0, 250, 500, 250), self.center_line)
                    self.ui.map_graphicsView.setScene(self.map_scene)

                    self.map_scene.addEllipse(x - int(self.diameter),
                                              y - int(self.diameter/2), 
                                              self.diameter,
                                              self.diameter,
                                              self.vehicle_pen)
                    self.pixmap = self.ui.map_graphicsView.grab(QRect(QPoint(0,0),QSize(500, 500)))


                    points = [[x - self.C*math.sin(angle), y - self.C*math.cos(angle)], 
                              [x - math.cos(angle)*self.C/2 + math.sqrt(3)*math.sin(angle)*self.C/2,
                               y + math.sin(angle)*self.C/2 + math.sqrt(3)*math.cos(angle)*self.C/2],
                              [x + math.cos(angle)*self.C/2 + math.sqrt(3)*math.sin(angle)*self.C/2,
                               y - math.sin(angle)*self.C/2 + math.sqrt(3)*math.cos(angle)*self.C/2]]

                    qpoly = QPolygonF([QPointF(p[0], p[1]) for p in points])
                    self.map_scene.addPolygon(qpoly, QPen(Qt.blue), QBrush(Qt.blue)) 

                    # Draw odometry position in green if available
                    if self.odom_x is not None and self.odom_y is not None:
                        self.map_scene.addEllipse(self.odom_x - int(self.diameter),
                                                  self.odom_y - int(self.diameter/2),
                                                  self.diameter,
                                                  self.diameter,
                                                  self.odom_pen,
                                                  QBrush(Qt.darkGreen))
                    # Draw big circle around robot
                    self.map_scene.addEllipse(x - self.circle_radius, y - self.circle_radius, 2*self.circle_radius, 2*self.circle_radius, QPen(Qt.darkMagenta))
        
        
    def update(self): 
        rclpy.spin_once(self.ros_node, timeout_sec=0)

    def publish_path(self):
        #'''
        path_to_publish = []
        path_to_publish.append(np.array(self.map_scene.path[0]))
        nodes_interval = 0.04 # meter
        path_index = 1
        path_to_publish_index = 0

        # removing points that are closer to each other than 0.04 meter.
        while True:
            have_removed_idex = False
            for i in range(len(self.map_scene.path)-1):
                distance = math.sqrt((self.map_scene.path[i+1][0] - self.map_scene.path[i][0])**2 +
                             (self.map_scene.path[i+1][1] - self.map_scene.path[i][1])**2)
                if(distance < 0.2):
                    self.map_scene.path.pop(i+1)
                    have_removed_idex = True
                    print(f"poped index{i+1}")
                    break
            if(have_removed_idex == False):
                print("all edges with less then 0.04 length are removed.")
                break

        while True:
            dist = math.sqrt((path_to_publish[path_to_publish_index][0] - self.map_scene.path[path_index][0])**2 +
                             (path_to_publish[path_to_publish_index][1] - self.map_scene.path[path_index][1])**2)

            if(dist >= nodes_interval):
                if((self.map_scene.path[path_index-1][0] - self.map_scene.path[path_index][0]) != 0):
                    grad = (self.map_scene.path[path_index-1][1] - self.map_scene.path[path_index][1])/(self.map_scene.path[path_index-1][0] - self.map_scene.path[path_index][0])
                    x1 = nodes_interval/math.sqrt(1 + grad**2) + path_to_publish[path_to_publish_index][0]
                    x2 = -nodes_interval/math.sqrt(1 + grad**2) + path_to_publish[path_to_publish_index][0]

                    p1 = np.array([x1, (x1 - path_to_publish[path_to_publish_index][0])*grad + path_to_publish[path_to_publish_index][1]])
                    p2 = np.array([x2, (x2 - path_to_publish[path_to_publish_index][0])*grad + path_to_publish[path_to_publish_index][1]])
                    path_target_node = np.array(self.map_scene.path[path_index])
                    A = path_target_node - path_to_publish[path_to_publish_index]
                    B = p2 - path_to_publish[path_to_publish_index]
                    C = p1 - path_to_publish[path_to_publish_index]
                    cos_theta1 = A@B/(np.linalg.norm(A)*np.linalg.norm(B))
                    cos_theta2 = A@C/(np.linalg.norm(A)*np.linalg.norm(C))
                    if(cos_theta1 < cos_theta2):
                        path_to_publish.append(p1)
                        path_to_publish_index += 1
                    else:
                        path_to_publish.append(p2)
                        path_to_publish_index += 1     
                else:
                    x = self.map_scene.path[path_index-1][0]
                    y1 = path_to_publish[path_to_publish_index][1] + 0.2
                    y2 = path_to_publish[path_to_publish_index][1] - 0.2

                    if(path_to_publish[path_to_publish_index][1] <= y1 <= self.map_scene.path[path_index][1]):
                        path_to_publish.append(np.array([x,y1]))
                    else:
                        path_to_publish.append(np.array([x,y2]))

                    path_to_publish_index += 1
            else:
                path_index += 1

            if path_index == len(self.map_scene.path)-1:
                self.pixmap = self.ui.map_graphicsView.grab(QRect(QPoint(0,0),QSize(500, 500)))
                self.enable_repaint = True
                break

            if path_to_publish_index > 10000:
                self.map_scene.path.clear()
                self.map_scene.clear()
                path_to_publish.clear()
                self.map_scene.prev_x = 250
                self.map_scene.prev_y = 250
                self.map_scene.path.append([(self.map_scene.prev_x - self.map_scene.center_x)*self.map_scene.scale, 
                                  -1*(self.map_scene.prev_y - self.map_scene.center_y)*self.map_scene.scale])
                self.map_scene.addLine(QLineF(250, 0, 250, 500), self.center_line) 
                self.map_scene.addLine(QLineF(0, 250, 500, 250), self.center_line)
                print("path generation failed. Please draw another path.")
                break

        print(path_to_publish)
        path_to_publish_np = np.array(path_to_publish)
        print(f"done!")
        path_pub = Float64MultiArray(data=np.ravel(path_to_publish_np))    
        self.pub.publish(path_pub) 

    def pixmap_to_cv(self, pixmap):
        qimage = pixmap.toImage()
        w, h, d = qimage.size().width(), qimage.size().height(), qimage.depth()
        bytes_ = qimage.bits().asstring(w * h * d // 8)
        arr = np.frombuffer(bytes_, dtype=np.uint8).reshape((h, w, d // 8))
        im_bgr = cv2.cvtColor(arr, cv2.COLOR_BGRA2BGR)
        return im_bgr
    
    def update_path_image(self):
        pixmap2cv = self.ui.map_graphicsView.grab(QRect(QPoint(1,1),QSize(500, 500)))
        frame = self.pixmap_to_cv(pixmap2cv)
        frame = cv2.resize(frame, (1000, 1000))
        cv2.imwrite(os.environ['HOME'] + 'ros/motion-planning/path_tracking_sim_ros2/src/robot_simulation/robot_gazebo/worlds/path_tracking_stage/meshes/stage.png', frame)
        install_dir = Path(os.environ['HOME'] + 'ros/motion-planning/path_tracking_sim_ros2/install/robot_gazebo/share/robot_gazebo/worlds/path_tracking_stage/meshes')
        if install_dir.exists():
            cv2.imwrite(os.environ['HOME'] + 'ros/motion-planning/path_tracking_sim_ros2/install/robot_gazebo/share/robot_gazebo/worlds/path_tracking_stage/meshes/stage.png', frame)

    def enable_draw_path(self):
        self.map_scene.set_mode('path')

    def enable_draw_obstacle(self):
        self.map_scene.set_mode('obstacle')

    def clear_obstacles(self):
        self.map_scene.clear_obstacles()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = Path_Tracking_Simulator()
    window.show()
    sys.exit(app.exec_())