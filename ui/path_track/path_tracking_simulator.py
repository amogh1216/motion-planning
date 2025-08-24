#! /usr/bin/python3
# -*- coding: utf-8 -*-

import sys
import os
import math
import numpy as np
import cv2
from pathlib import Path
import yaml

from PyQt5.Qt import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from path_tracking_simulator_window import Ui_Form

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseStamped

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
    def __init__(self, parent=None):

        super(Path_Tracking_Simulator, self).__init__(parent)
        self.ui = Ui_Form()
        self.ui.setupUi(self)

        # Load parameters from YAML file
        params_file = Path(__file__).parent / 'ui_simulator_params.yaml'
        with open(params_file, 'r') as file:
            params = yaml.safe_load(file)

        # Map parameters
        map_params = params['map']
        self.map_scene = Map_GraphicsScene(self.ui.map_graphicsView)
        self.map_scene.scale = map_params['scale']
        self.map_scene.center_x = map_params['center_x']
        self.map_scene.center_y = map_params['center_y']
        self.ui.map_graphicsView.setScene(self.map_scene)

        # Lidar parameters
        lidar_params = params['lidar']
        self.circle_radius = lidar_params['circle_radius']
        self.num_rays = lidar_params['num_rays']
        self.effective_range = lidar_params['effective_range']
        self.point_cloud_stdev = lidar_params['point_cloud_stdev'] / self.map_scene.scale
        self.lidar_update_interval = lidar_params['update_interval']
        self.build_map = lidar_params['build_map']
        self.estimation_mode = lidar_params['estimation_mode'] 

        self.lidar_scene = QGraphicsScene(self.ui.map_lidarView)
        self.ui.map_lidarView.setScene(self.lidar_scene)

        # Robot parameters
        robot_params = params['robot']
        self.diameter = robot_params['diameter']
        self.C = robot_params['length']

        # ROS parameters
        ros_params = params['ros']
        rclpy.init(args=None)
        self.ros_node = Node(ros_params['node_name'])
        self.pub = self.ros_node.create_publisher(Float64MultiArray, ros_params['path_topic'], 10)
        self.sub = self.ros_node.create_subscription(TFMessage, ros_params['tf_topic'], self.listener_callback, 10)
        self.sub_odom = self.ros_node.create_subscription(PoseStamped, ros_params['pose_topic'], self.odom_callback, 10)

        # Path generation parameters
        path_params = params['path']
        self.nodes_interval = path_params['nodes_interval']
        self.max_path_size = path_params['max_path_size']

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update)
        self.timer.start(10)

        self.pixmap = QPixmap(500, 500)
        self.vehicle_pen = QPen(Qt.red)
        self.center_line = QPen(Qt.black)
        self.center_line.setStyle(Qt.DashLine)
        self.map_scene.addLine(QLineF(250, 0, 250, 500), self.center_line)
        self.map_scene.addLine(QLineF(0, 250, 500, 250), self.center_line)

        self.first_time = True
        self.enable_repaint = False
        self.odom_pose_2d = [None, None, None]  # [x, y, angle]
        self.odom_pen = QPen(Qt.darkGreen)
        self.truth_pose_2d = [0, 0, 0]
        self.prev_truth_pose_2d = self.truth_pose_2d
        self.odom_callback_count = 0
        self.odom_cum_error = 0
        self.odom_time_since = QElapsedTimer()
        self.odom_time_since.start()
        self.truth_time_since = QElapsedTimer()
        self.truth_time_since.start()

        self.ui.draw_path_pushButton.clicked.connect(self.enable_draw_path)
        self.ui.draw_obstacle_pushButton.clicked.connect(self.enable_draw_obstacle)
        self.ui.clear_obstacles_pushButton.clicked.connect(self.clear_obstacles)

        self.rays = []
        self.point_cloud = []

        self.lidar_update_timer = QElapsedTimer()
        self.lidar_update_timer.start()

        # Timer to call make_scene every 30 ms
        self.scene_timer = QTimer(self)
        self.scene_timer.timeout.connect(self.make_scene)
        self.scene_timer.start(30)

    def odom_callback(self, msg):
        # Extract x, y, heading (imu) from Odometry message
        self.odom_pose_2d[0] = msg.pose.position.x / self.map_scene.scale + self.map_scene.center_x
        self.odom_pose_2d[1] = -msg.pose.position.y / self.map_scene.scale + self.map_scene.center_y
        self.odom_pose_2d[2] = msg.pose.position.z
        
        #print(f'ODOM  | since last: {self.odom_time_since.elapsed()} ')
        self.odom_time_since.restart()
        

    def listener_callback(self, data):
        for tfsf in data.transforms:
            if(tfsf.child_frame_id == 'rwd_bot'):
                #print(f'TRUTH | since last: {self.truth_time_since.elapsed()} ')
                self.truth_time_since.restart()

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

                self.prev_truth_pose_2d = self.truth_pose_2d
                self.truth_pose_2d = [x, y, angle]

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

        path_to_publish_np = np.array(path_to_publish)
        path_pub = Float64MultiArray(data=np.ravel(path_to_publish_np))    
        self.pub.publish(path_pub)

    def enable_draw_path(self):
        self.map_scene.set_mode('path')

    def enable_draw_obstacle(self):
        self.map_scene.set_mode('obstacle')

    def clear_obstacles(self):
        self.map_scene.clear_obstacles()

    def make_scene(self):
        if self.enable_repaint:
            if not self.estimation_mode:
                x, y, angle = self.truth_pose_2d
            else:
                x, y, angle = self.odom_pose_2d
            
            if x is None or y is None or angle is None:
                print(f"One or more of x, y, angle values are None.")
                return

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

            # Draw odometry position in green if available
            if all(value is not None for value in self.odom_pose_2d):

                err = math.sqrt((self.odom_pose_2d[0] - self.truth_pose_2d[0])**2 + 
                                (self.odom_pose_2d[1] - self.truth_pose_2d[1])**2)
                
                self.odom_callback_count += 1
                self.odom_cum_error += err
                print(f'odom error: {err}, avg err {self.odom_cum_error/self.odom_callback_count}')

                dark_green = QColor(Qt.darkGreen)
                dark_green.setAlpha(100)
                self.draw_robot(self.odom_pose_2d[0], self.odom_pose_2d[1], self.odom_pose_2d[2] - math.pi/2, dark_green)

            self.point_cloud = self.update_lidar_point_cloud()
            self.draw_point_cloud_estimate(x, y)

            # draw blue robot position + orientation
            self.draw_robot(self.truth_pose_2d[0], self.truth_pose_2d[1], self.truth_pose_2d[2], Qt.blue)
            # Draw a circle of 500 pixel diameter around the true robot position
            rad = self.circle_radius
            self.map_scene.addEllipse(
                self.truth_pose_2d[0] - rad, self.truth_pose_2d[1] - rad, 2*rad, 2*rad,
                QPen(Qt.darkGray, 1, Qt.DashLine)
            )

    def add_lidar_noise(self, point):
        if len(point) != 2:
            raise ValueError("Point must be a list with two elements [x, y].")

        # Generate noise from a normal distribution centered at 0
        noise_x = np.random.normal(0, self.point_cloud_stdev)
        noise_y = np.random.normal(0, self.point_cloud_stdev)

        # Add noise to the original point
        x_noisy = point[0] + noise_x
        y_noisy = point[1] + noise_y

        return [x_noisy, y_noisy]

    # returns relative points from pose estimate
    def update_lidar_point_cloud(self):
        
        x, y, angle = self.truth_pose_2d
        if x is None or y is None or angle is None:
            print(f'Can not update point cloud when true robot pose has item(s) of None.')
            return
        
        relative_point_cloud = []
        self.rays.clear()

        if (not self.build_map):
            self.lidar_scene.clear()  # Clear the lidar view before updating, relative points
        
        # Only update lidar point cloud if the timer has reached the interval
        if self.lidar_update_timer.elapsed() >= self.lidar_update_interval:
            update_cloud = True
            self.lidar_update_timer.restart()
        else: 
            update_cloud = False

        # Draw lines every X degrees from robot position, first line in robot's heading - range/2
        for i in range(self.num_rays+1):
            theta = -angle - math.pi/2 - self.effective_range/2 + (i * self.effective_range / self.num_rays) 
            # first line = heading, then every 60 deg
            x2 = x + self.circle_radius * math.cos(theta)
            y2 = y + self.circle_radius * math.sin(theta)
            pt = self.map_scene.get_single_ray_obstacle_intersections([x, y, x2, y2], self.map_scene.obstacles)
            semi_transparent_magenta = QColor(Qt.darkMagenta)
            semi_transparent_magenta.setAlpha(40)
            if pt is None:
                # draw lidar line 
                self.map_scene.addLine(x, y, x2, y2, QPen(semi_transparent_magenta, 2))
            else:
                rel_pt = [pt[0] - x, pt[1] - y]
                rel_pt = self.add_lidar_noise(rel_pt)
                pt = self.add_lidar_noise(pt)
                # draw lidar line to intersecting point
                self.map_scene.addLine(x, y, pt[0], pt[1], QPen(semi_transparent_magenta, 2))
                # Add the intersection point to the lidar view every self.lidar_update_interval ms
                if update_cloud:
                    # self.point_cloud.append(rel_pt)
                    relative_point_cloud.append(rel_pt)
                    # draws true map from lidar
                    if self.build_map:
                        self.lidar_scene.addEllipse(pt[0] - 2, pt[1] - 2, 4, 4, QPen(Qt.black), QBrush(Qt.red))
            
            self.rays.append([x, y, x2, y2])
        return relative_point_cloud
    
    def draw_point_cloud_estimate(self, x, y):
        if len(self.point_cloud) == 0:
            return
        
        for pt in self.point_cloud:
            mapped_pt = [x + pt[0], y + pt[1]]
            self.lidar_scene.addEllipse(mapped_pt[0] - 2, mapped_pt[1] - 2, 4, 4, QPen(Qt.red), QBrush(Qt.red))


    def draw_robot(self, x, y, angle, color):
        points = [[x - self.C*math.sin(angle), y - self.C*math.cos(angle)], 
                              [x - math.cos(angle)*self.C/2 + math.sqrt(3)*math.sin(angle)*self.C/2,
                               y + math.sin(angle)*self.C/2 + math.sqrt(3)*math.cos(angle)*self.C/2],
                              [x + math.cos(angle)*self.C/2 + math.sqrt(3)*math.sin(angle)*self.C/2,
                               y - math.sin(angle)*self.C/2 + math.sqrt(3)*math.cos(angle)*self.C/2]]
        qpoly = QPolygonF([QPointF(p[0], p[1]) for p in points])
        self.map_scene.addPolygon(qpoly, QPen(color), QBrush(color))


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = Path_Tracking_Simulator()
    window.show()
    sys.exit(app.exec_())