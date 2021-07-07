#!/usr/bin/env python
"""
@@@ warning @@@
U can control the local position of the drone
actually the problem is when u exit the controller
then from next time u run this then the local x, y pos are different
so u need to know if u run this again then u need to check 
the local position of the drone and fix it near to the origin local position
"""
from __future__ import print_function

import threading

import roslib
import rospy
import mavros
import time

from mavros_msgs.msg import PositionTarget
from mavros_msgs.srv import SetMode, CommandBool
from std_msgs.msg import String, Int8
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import PoseStamped

import sys, select, termios, tty



moveTab = {
        'GO':(-1,0,0,0),
        'BACK':(1,0,0,0),
        'LEFT':(0,-1,0,0),
        'RIGHT':(0,1,0,0),
        'GLEFT':(-1,-1,0,0),
        'GRIGHT':(-1,1,0,0),
        'BLEFT':(1,-1,0,0),
        'BRIGHT':(1,1,0,0),
        'UP':(0,0,1,0),
        'DOWN':(0,0,-1,0),
        'STOP':(0,0,0,0), 
        'ARM':(0,0,0,0),
}

switch = ""
zheight = 0
past = ""
pubx = rospy.Publisher('drone/position/x', String, queue_size=3)
puby = rospy.Publisher('drone/position/y', String, queue_size=3)
pubz = rospy.Publisher('drone/position/z', String, queue_size=3)
pubox = rospy.Publisher('drone/position/ox', String, queue_size=3)
puboy = rospy.Publisher('drone/position/oy', String, queue_size=3)
puboz = rospy.Publisher('drone/position/oz', String, queue_size=3)
pubow = rospy.Publisher('drone/position/ow', String, queue_size=3)

pubBattery = rospy.Publisher('drone/battery', String, queue_size=3)

def getKey(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def arming(bool_):
    """
    arming(0) >> disarming
    arming(1) > arming
    """
    arming = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    arming(value=bool_)

def set_mode(mode):
    """
    There are many modes u can set
    set_mode("OFFBOARD") >> Offboard mode
    
    MANUAL, ACRO, ALTCTL, POSCTL,
    OFFBOARD, STABILIZED, RATTITUDE,
    AUTO.MISSION ,AUTO.LOITER, AUTO.RTL,
    AUTO.LAND, AUTO.RTGS, AUTO.READY,
    AUTO.TAKEOFF 
    """
    set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
    set_mode(0, mode)
    
class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=3)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, x, y, z, yaw):
        self.condition.acquire()
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0)
        self.join()

    def run(self):
        pos = PositionTarget()
        pos.coordinate_frame = 1
        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            # Copy state into twist message.
            pos.position.x = pos_x
            pos.position.y = pos_y
            pos.position.z = zheight
            pos.yaw = yaw
            self.condition.release()

            self.publisher.publish(pos)

            
        pos.position.x = 0
        pos.position.y = 0
        pos.position.z = 0
        pos.yaw = 0

        self.publisher.publish(pos)


def callback(msg):
    global switch 
    switch = msg.data

def zcallback(msg):
    global zheight 
    zheight = int(msg.data)/100

def pos_callback(msg):
    px = msg.pose.position.x
    py = msg.pose.position.y
    pz = msg.pose.position.z
    ox = msg.pose.orientation.x
    oy = msg.pose.orientation.y
    oz = msg.pose.orientation.z
    ow = msg.pose.orientation.w
    #print("get")
    pubx.publish(str(round(px,1)))
    puby.publish(str(round(py,1)))
    pubz.publish(str(round(pz,1)))
    pubox.publish(str(round(ox,1)))
    puboy.publish(str(round(oy,1)))
    puboz.publish(str(round(oz,1)))
    pubow.publish(str(round(ow,1)))


def batt_callback(msg):
    pb = int(msg.percentage*100)
    pubBattery.publish(str(pb))



if __name__=="__main__":
    msg_cnt = 0
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('teleop_setpoint_keyboard')
    key_timeout = rospy.get_param("~key_timeout", 0.0)
    if key_timeout == 0.0:
        key_timeout = None

    rospy.Subscriber('commandar_line', String, callback)
    rospy.Subscriber('zcontrol', String, zcallback)
    rospy.Subscriber('mavros/local_position/pose', PoseStamped, pos_callback)
    rospy.Subscriber('mavros/battery', BatteryState, batt_callback)

    repeat = rospy.get_param("~repeat_rate", 20)
    pub_thread = PublishThread(repeat)
    pos_x = 0.0
    pos_y = 0.0
    pos_z = 0.0
    yaw = 0.0

    txt = ""
    txt += 'x : ' + str(pos_x)
    txt += ', y : ' + str(pos_y)
    txt += ', z : ' + str(pos_z)
    txt += ', deg : ' + str(yaw*180/3.1415)
    print(txt, end="")

    


    try:
        pub_thread.wait_for_subscribers()
        print(zheight)
        pub_thread.update(pos_x, pos_y, zheight, yaw)
        
        while(1):
            if switch == 'TO':
                set_mode("OFFBOARD")
            if switch == 'ARM':
                arming(1)
            if switch == 'ARM_OFF':
                arming(0)
            if switch == 'TO_OFF':
                break
            if switch == '':
                switch = past

            if switch in moveTab.keys():
                pos_x += moveTab[switch][0] * 0.02
                pos_y += moveTab[switch][1] * 0.02
                pos_z += moveTab[switch][2] * 0.02
                yaw += moveTab[switch][3] * 3.1415 / 20          
                if pos_z <= 0 : pos_z = 0
                past = switch
                time.sleep(0.01)

            if switch != '':
                print('pos_x : ',pos_x)
                print('pos_y : ',pos_y)
                print('pos_z : ',pos_z)
                print('yaw : ',yaw*180/3.1415)            
            
            pub_thread.update(pos_x, pos_y, zheight, yaw)

            
    except Exception as e:
        print(e)

    finally:

        pub_thread.stop()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
