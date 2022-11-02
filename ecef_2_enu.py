#!/usr/bin/env python2
# license removed for brevity

import numpy as np
import math
import rospy
import time
from std_msgs.msg import Float64MultiArray
from sbg_driver.msg import SbgGpsPos
from geometry_msgs.msg import PointStamped

class ecef_2_enu : 

    def __init__(self):
        rospy.init_node('ecef2enu',anonymous=True)
        rospy.Subscriber("/sbg/gps_pos",SbgGpsPos,self.callback_latlong)
        rospy.Subscriber("/imu/pos_ecef",PointStamped,self.callback_ecef)

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        self.lat = 0.0
        self.lon = 0.0


    def callback_latlong(self, data):
        # print("callback_latlong is on...")
        
        self.lat = math.radians(data.latitude)
        self.lon = math.radians(data.longitude)

        # print("latitude [rad] : ", lat)
        # print("longitude [rad]: ", lon)

    
    def callback_ecef(self, data):
        # print("callback_ecef is on...")

        self.x = data.point.x
        self.y = data.point.y
        self.z = data.point.z

        # print("x : ", x)
        # print("y : ", y)
        # print("z : ", z)


    def ecef2enu(self):

        print("ecef2enu is on...")

        x0 = 4143456.30851996 
        y0 = 611600.8746296273 
        z0 = 4794255.290430106

        dx = self.x-x0 
        dy = self.y-y0 
        dz = self.z-z0

        dxyz = np.array([dx,dy,dz])
        R = np.array([[-math.sin(self.lat)*math.cos(self.lon),-math.sin(self.lat)*math.sin(self.lon),math.cos(self.lat)],
        [-math.sin(self.lon), math.cos(self.lon),0],
        [math.cos(self.lat)*math.cos(self.lon),math.cos(self.lat)*math.sin(self.lon),math.sin(self.lat)]])

        dxyz = np.transpose(dxyz)
        enu = np.dot(R,dxyz)    

        e = enu[0] 
        n = enu[1] 
        u = enu[2] 
        print("e : ", e)
        print("n : ", n)
        print("u : ", u)

        # return e, n, u   


if __name__ == "__main__":
    enu = ecef_2_enu()
    while not rospy.is_shutdown():
        enu.ecef2enu()


