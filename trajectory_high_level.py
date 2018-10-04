#!/usr/bin/env python

import rospy
import tf
import crazyflie
import time
from crazyflie_driver.msg import Position
from std_msgs.msg import Empty
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Joy
from crazyflie_driver.srv import UpdateParams
import numpy as np
import pandas as pd
from threading import Thread


def handler(cf, tr):
    cf.setParam("commander/enHighLevel", 1)
    #time.sleep(1)

    HEIGHT = 1.0
    cf.takeoff(targetHeight = HEIGHT, duration = 2.0)
    time.sleep(3.0)
    
    rospy.loginfo("Executing trajectory")
    x = tr[:,0]; y = tr[:,1]; t = tr[:,2]
    for i in range(len(x)):
        rospy.loginfo(str(cf.prefix)+' Moving to: '+str(x[i])+', '+str(y[i])+' time: '+str(t[i]+0.1))
        try:
            if joy.buttons[1]==1:
                break
        except:
            cf.goTo(goal = [x[i], y[i], HEIGHT], yaw=0.0, duration = t[i]+0.1, relative = False)
            time.sleep(t[i]+0.1)

    cf.land(targetHeight = -0.1, duration = 2.0)
    time.sleep(3.0)

    cf.stop()

if __name__ == '__main__':
    rospy.init_node('trajectory', anonymous=True)
    worldFrame = rospy.get_param("~worldFrame", "/world")

    trajectory0 = np.array( pd.read_csv('~/Desktop/crazyflies/trajectory0.csv') )
    trajectory1 = np.array( pd.read_csv('~/Desktop/crazyflies/trajectory1.csv') )
    trajectory2 = np.array( pd.read_csv('~/Desktop/crazyflies/trajectory2.csv') )

    cf18 = crazyflie.Crazyflie("crazyflie18", "/vicon/crazyflie18/crazyflie18")
    cf15 = crazyflie.Crazyflie("crazyflie15", "/vicon/crazyflie15/crazyflie15")
    cf13 = crazyflie.Crazyflie("crazyflie13", "/vicon/crazyflie13/crazyflie13")

    rate = rospy.Rate(10) # 10 hz

    msg = Position()
    msg.header.seq = 0
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = worldFrame
    msg.x = 0.0
    msg.y = 0.0
    msg.z = 0.0
    msg.yaw = 0.0

    t1 = Thread(target=handler, args=(cf18, trajectory0))
    t2 = Thread(target=handler, args=(cf15, trajectory1))
    t3 = Thread(target=handler, args=(cf13, trajectory2))

    t1.start()
    t2.start()
    t3.start()