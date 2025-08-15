#! /usr/bin/python3
# -*- coding: utf-8 -*-

import sys
import math

from PyQt5.Qt import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from path_visualization_window import Ui_Form

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from tf2_msgs.msg import TFMessage

class Map_GraphicsScene(QGraphicsScene):
    def __init__(self, parent=None):
        QGraphicsScene.__init__(self, parent)
        self.center_x = 250
        self.center_y = 250
        self.prev_x = 250
        self.prev_y = 250
        self.scale = 0.04 #m/pix

class Path_Visualization_Simulator(QDialog):
    def __init__(self,parent=None):
        super(Path_Visualization_Simulator, self).__init__(parent)
        self.ui = Ui_Form()
        self.ui.setupUi(self)

        self.map_scene = Map_GraphicsScene(self.ui.map_graphicsView)
        self.ui.map_graphicsView.setScene(self.map_scene)

        rclpy.init(args=None)
        self.sub_node = Node('sub_observation')
        # visualize ground truth gazebo movement
        self.sub = self.sub_node.create_subscription(TFMessage, '/world/my_world/pose/info', self.listener_callback, 10)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update)
        self.timer.start(10)
        
        self.pixmap = QPixmap(500, 500)
        self.vehicle_pen = QPen(Qt.red)
        self.center_line=QPen(Qt.black)
        self.center_line.setStyle(Qt.DashLine)
        self.map_scene.addLine(QLineF(250, 0, 250, 500), self.center_line) 
        self.map_scene.addLine(QLineF(0, 250, 500, 250), self.center_line)
        self.diameter = 2 #pix
        self.C = 10 #length
        self.first_time = True

        self.repaint(75, 75)

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

                if(self.first_time == False):
                    self.map_scene.clear()
                    self.map_scene.addPixmap(self.pixmap)
                    self.map_scene.addLine(QLineF(250, 0, 250, 500), self.center_line) 
                    self.map_scene.addLine(QLineF(0, 250, 500, 250), self.center_line)
                    self.ui.map_graphicsView.setScene(self.map_scene)
                    self.first_time = False

                self.map_scene.addEllipse(x - int(self.diameter), y - int(self.diameter/2), 
                                      self.diameter, self.diameter,
                                      self.vehicle_pen)
                self.pixmap = self.ui.map_graphicsView.grab(QRect(QPoint(0,0),QSize(500, 500)))


                # create triangle on UI
                points = [[x - self.C*math.sin(angle), y - self.C*math.cos(angle)], 
                          [x - math.cos(angle)*self.C/2 + math.sqrt(3)*math.sin(angle)*self.C/2,
                           y + math.sin(angle)*self.C/2 + math.sqrt(3)*math.cos(angle)*self.C/2],
                          [x + math.cos(angle)*self.C/2 + math.sqrt(3)*math.sin(angle)*self.C/2,
                           y - math.sin(angle)*self.C/2 + math.sqrt(3)*math.cos(angle)*self.C/2]]

                qpoly = QPolygonF([QPointF(p[0], p[1]) for p in points])
                self.map_scene.addPolygon(qpoly, QPen(Qt.blue), QBrush(Qt.blue)) 
        
        
    def update(self):
        rclpy.spin_once(self.sub_node)
      
    def repaint(self, x, y):
        if(self.first_time):
            self.first_time = False

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = Path_Visualization_Simulator()
    window.show()
    sys.exit(app.exec_())