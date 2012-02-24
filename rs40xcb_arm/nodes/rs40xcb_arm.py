#!/usr/bin/env python
import roslib; roslib.load_manifest('rs40xcb_arm')
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray

from rs40xcb_python import rs40xcb


set_time = []
def trqCallback(trqData):
    j=0
    for i in range(startID,startID+len(trqData.data)):
#        print(i)
        if trqData.data[j] == 1:
            servo.torque(i,1)
#            print("ON")
        else:
            servo.torque(i,0)
#            print("OFF")
        j+=1
        
    rospy.loginfo("Torque set")
    

def posCallback(posData):
    j=0
    for i in range(startID,startID+len(posData.data)):
        servo.move(i,posData.data[j],set_time[j])
#        print(i)
#        print(posData.data[j])
#        print(set_time[j])
        j+=1
        
    rospy.loginfo("Move set")
    

def timeCallback(timeData):
    global set_time
    set_time = []
    set_time = timeData.data
    
    rospy.loginfo("Time set")


def talker():
    
    time_pub = rospy.Publisher('ArmTimeGet', Int32MultiArray)
    vlt_pub = rospy.Publisher('ArmVoltageGet', Int32MultiArray)
    speed_pub = rospy.Publisher('ArmSpeedGet', Int32MultiArray)
    load_pub = rospy.Publisher('ArmLoadGet', Int32MultiArray)
    pos_pub = rospy.Publisher('ArmPositionGet', Int32MultiArray)
    temp_pub = rospy.Publisher('ArmTemperatureGet', Int32MultiArray)
    
    rospy.Subscriber("ArmTorqueSet", Int32MultiArray, trqCallback)
    rospy.Subscriber("ArmPositionSet", Int32MultiArray, posCallback)
    rospy.Subscriber("ArmTimeSet", Int32MultiArray, timeCallback)
    
    
    while not rospy.is_shutdown():
        time_buf =Int32MultiArray()
        vlt_buf = Int32MultiArray()
        speed_buf = Int32MultiArray()
        pos_buf = Int32MultiArray()
        load_buf = Int32MultiArray()
        temp_buf = Int32MultiArray()
        for i in range(startID,endID+1):
            servo.getParam(i)
            pos_buf.data.append(servo.getAngle(i))
            time_buf.data.append(servo.getTime(i))
            load_buf.data.append(servo.getLoad(i))
            temp_buf.data.append(servo.getTemperature(i))
            speed_buf.data.append(servo.getSpeed(i))
            vlt_buf.data.append(servo.getVoltage(i))
        
        time_pub.publish(time_buf)
        vlt_pub.publish(vlt_buf)
        speed_pub.publish(speed_buf)
        pos_pub.publish(pos_buf)
        load_pub.publish(load_buf)    
        temp_pub.publish(temp_buf)

        rospy.sleep(0.05)
        
if __name__ == '__main__':
    rospy.init_node('rs40xcb_arm')
    rospy.loginfo("in")
    
    portname = rospy.get_param('~port','/dev/ttyUSB0')
    baud = int(rospy.get_param('~baud','115200'))
    startID = int(rospy.get_param('~startID','1'))
    endID = int(rospy.get_param('~endID','7'))
    
    servo = rs40xcb.RS40XCB(portname,baud)
    
    for i in range(endID-startID+1):
        set_time.append(0)
#        print(i)
        
    try:
        talker()
    except rospy.ROSInterruptException: pass
