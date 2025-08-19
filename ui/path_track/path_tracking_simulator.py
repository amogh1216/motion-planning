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
        self.obstacles = []  # 2D array: [[x1, y1, x2, y2], ...]

    def set_mode(self, mode):
        self.mode = mode
        if mode == 'path':
            self.clear_path()
        # Do not clear obstacles when switching to obstacle mode

    def clear_path(self):
        for item in self.path_items:
            self.removeItem(item)
        self.path_items.clear()
        self.path.clear()
        self.prev_x = self.center_x
        self.prev_y = self.center_y
        self.path.append([(self.prev_x - self.center_x)*self.scale, 
                     -1*(self.prev_y - self.center_y)*self.scale])

    def clear_obstacles(self):
        for item in self.obstacle_items:
            self.removeItem(item)
        self.obstacle_items.clear()
        self.obstacles.clear()

    def mouseMoveEvent(self,event):
        x = event.scenePos().x()
        y = event.scenePos().y()
        
        if self.mode == 'path':
            dist_sqd = (self.prev_x - x)**2 + (self.prev_y - y)**2
            # ensures path resolution isn't too small while drawing
            if dist_sqd > (self.scale ** 2):
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
            # Update or add the current obstacle in the array
            if len(self.obstacles) > 0:
                self.obstacles[-1] = [self.prev_x, self.prev_y, x, y]
            print(f"obstacle at idx {len(self.obstacles)-1} is {self.obstacles[-1]}")

    def mousePressEvent(self, event):
        x = event.scenePos().x()
        y = event.scenePos().y()
        if self.mode == 'path':
            if len(self.path) == 0:
                self.clear_path()
            if self.drawing_path:
                # If already drawing, clear previous path
                for item in self.path_items:
                    self.removeItem(item)
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
            # Start a new obstacle entry
            self.obstacles.append([self.prev_x, self.prev_y, self.prev_x, self.prev_y])

    def get_ray_obstacle_intersections(self, rays, obstacles):
        intersections = []
        for ray in rays:
            x1, y1, x2, y2 = ray
            for obs in obstacles:
                x3, y3, x4, y4 = obs
                denom = (x1-x2)*(y3-y4) - (y1-y2)*(x3-x4)
                if denom == 0:
                    continue  # Parallel lines
                px = ((x1*y2 - y1*x2)*(x3-x4) - (x1-x2)*(x3*y4 - y3*x4)) / denom
                py = ((x1*y2 - y1*x2)*(y3-y4) - (y1-y2)*(x3*y4 - y3*x4)) / denom
                # Check if intersection is within both segments
                if (min(x1, x2)-1e-6 <= px <= max(x1, x2)+1e-6 and
                    min(y1, y2)-1e-6 <= py <= max(y1, y2)+1e-6 and
                    min(x3, x4)-1e-6 <= px <= max(x3, x4)+1e-6 and
                    min(y3, y4)-1e-6 <= py <= max(y3, y4)+1e-6):
                    intersections.append((px, py))
        return intersections
    
    def get_single_ray_obstacle_intersections(self, ray, obstacles):
        x1, y1, x2, y2 = ray
        closest_dist = None
        closest_point = None
        for obs in obstacles:
            x3, y3, x4, y4 = obs
            denom = (x1-x2)*(y3-y4) - (y1-y2)*(x3-x4)
            if denom == 0:
                continue  # Parallel lines
            px = ((x1*y2 - y1*x2)*(x3-x4) - (x1-x2)*(x3*y4 - y3*x4)) / denom
            py = ((x1*y2 - y1*x2)*(y3-y4) - (y1-y2)*(x3*y4 - y3*x4)) / denom
            # Check if intersection is within both segments
            if (min(x1, x2)-1e-6 <= px <= max(x1, x2)+1e-6 and
                min(y1, y2)-1e-6 <= py <= max(y1, y2)+1e-6 and
                min(x3, x4)-1e-6 <= px <= max(x3, x4)+1e-6 and
                min(y3, y4)-1e-6 <= py <= max(y3, y4)+1e-6):
                dist = math.hypot(px - x1, py - y1)
                if closest_dist is None or dist < closest_dist:
                    closest_dist = dist
                    closest_point =  [px, py]
        if closest_point is not None:
            return closest_point
        else:
            return None

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
        self.diameter = 2 #pix
        self.C = 5 #length
        self.first_time = True
        self.enable_repaint = False
        self.odom_x = None
        self.odom_y = None
        self.odom_pen = QPen(Qt.darkGreen)

        self.ui.draw_path_pushButton.clicked.connect(self.enable_draw_path)
        self.ui.draw_obstacle_pushButton.clicked.connect(self.enable_draw_obstacle)
        self.ui.clear_obstacles_pushButton.clicked.connect(self.clear_obstacles)

        self.circle_radius = 50  # in pixels, default value, divided by scale = m
        self.num_rays = 6
        self.effective_range = math.pi
        self.rays = [] # 2D array: [[x1, y1, x2, y2], ...]
        self.point_cloud = [] # intersecting points from rays and obstacles
        # Optionally, load from params or config file

    def odom_callback(self, msg):
        # Extract x, y from Odometry message
        x = msg.pose.pose.position.x / self.map_scene.scale + self.map_scene.center_x
        y = -msg.pose.pose.position.y / self.map_scene.scale + self.map_scene.center_y
        self.odom_x = x
        self.odom_y = y

    def update_lidar_point_cloud(self, x, y, angle):
        self.point_cloud.clear()
        self.rays.clear()
        # Draw lines every X degrees from robot position, first line in robot's heading - range/2
        for i in range(self.num_rays+1):
            theta = -angle - math.pi/2 - self.effective_range/2 + (i * self.effective_range / self.num_rays) 
            # first line = heading, then every 60 deg
            x2 = x + self.circle_radius * math.cos(theta)
            y2 = y + self.circle_radius * math.sin(theta)
            pt = self.map_scene.get_single_ray_obstacle_intersections([x, y, x2, y2], self.map_scene.obstacles)
            semi_transparent_magenta = QColor(Qt.darkMagenta)
            semi_transparent_magenta.setAlpha(50)
            if pt is None:
                self.map_scene.addLine(x, y, x2, y2, QPen(semi_transparent_magenta, 2))
            else:
                self.point_cloud.append(pt)
                self.map_scene.addLine(x, y, pt[0], pt[1], QPen(semi_transparent_magenta, 2))
            self.rays.append([x, y, x2, y2])

        print(f"num intersections: {len(self.map_scene.get_ray_obstacle_intersections(self.rays, self.map_scene.obstacles))}")
        print(f"point cloud: {self.point_cloud}")

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
                denominator = 1 - 2 * (q1**2 + q2**2)
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

                    # red path tracker
                    self.map_scene.addEllipse(x - int(self.diameter),
                                              y - int(self.diameter/2), 
                                              self.diameter,
                                              self.diameter,
                                              self.vehicle_pen, QBrush(Qt.red))
                    self.pixmap = self.ui.map_graphicsView.grab(QRect(QPoint(0,0),QSize(500, 500)))

                    self.update_lidar_point_cloud(x, y, angle)

                    # draw blue robot position + orientation
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
                    
                    # Draw big circle around robot, lidar range
                    self.map_scene.addEllipse(x - self.circle_radius, y - self.circle_radius, 2*self.circle_radius, 2*self.circle_radius, QPen(Qt.darkMagenta))
        
    def update(self): 
        rclpy.spin_once(self.ros_node, timeout_sec=0)

    def publish_path(self):
        path_to_publish = []
        path_to_publish.append(np.array(self.map_scene.path[0]))
        nodes_interval = 0.04 # meter
        path_index = 1
        path_to_publish_index = 0
        print(f"len {len(self.map_scene.path)}")

        while True:            
            currpt = np.array(path_to_publish[path_to_publish_index])
            targetpt = np.array(self.map_scene.path[path_index])
            direction = targetpt - currpt
            distance = np.linalg.norm(direction)

            if(distance >= nodes_interval):
                unit_d = direction / distance
                nextpt = currpt + (unit_d * nodes_interval)
                path_to_publish.append(nextpt)
                path_to_publish_index += 1
            else:
                path_index += 1

            if path_index == len(self.map_scene.path)-1:
                self.pixmap = self.ui.map_graphicsView.grab(QRect(QPoint(0,0),QSize(500, 500)))
                self.enable_repaint = True
                break

            # upper limit to path size
            if path_to_publish_index > 10000:
                self.map_scene.clear_path()
                path_to_publish.clear()
                self.map_scene.addLine(QLineF(250, 0, 250, 500), self.center_line) 
                self.map_scene.addLine(QLineF(0, 250, 500, 250), self.center_line)
                print(f"path generation failed. Please draw another path.")
                path_to_publish_np = np.array(path_to_publish)
                path_pub = Float64MultiArray(data=np.ravel(path_to_publish_np))    
                self.pub.publish(path_pub) 
                return

        print(f"len ptp: {len(path_to_publish)}")
        path_to_publish_np = np.array(path_to_publish)
        print(f"done!")
        path_pub = Float64MultiArray(data=np.ravel(path_to_publish_np))    
        self.pub.publish(path_pub)

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