#!/usr/bin/env python3

import rospy
import serial
import utm
from std_msgs.msg import Header
from gps_driver.msg import gps_msg

def convert_to_utm(data):
    data = data/100
    degrees = int(data)
    minutes = data - degrees
    utm_data = degrees + (minutes*100)/60
    return utm_data

def gps_processor():
    rospy.init_node("gps_processor")
    pub = rospy.Publisher("/gps", gps_msg, queue_size=10)
    serial_port = rospy.get_param("~port", "/dev/ttyUSB0")
    baud_rate  = rospy.get_param("~baud_rate", 4800)
    gps_signal = serial.Serial(serial_port, baud_rate)
    msg = gps_msg()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        data = str(gps_signal.readline())
        # print("data")
        datalist = data.split(",")
        print(datalist[0])
        if datalist[0] == "b'$GPGGA":
            # print("hello world")
            if datalist[2] == '':
                continue
            elif datalist[4] =='':
                continue
            else:
                msg.header = Header()
                msg.header.frame_id = "GPS1_Frame"
                gps_time = float(datalist[1])
                msg.header.stamp.secs = int(gps_time)
                msg.header.stamp.nsecs = int((gps_time - msg.header.stamp.secs) * 1e9)
                msg.latitude = convert_to_utm(float(datalist[2]))
                msg.longitude = convert_to_utm(-float((datalist[4])))
                # position_utm = np.array(utm.from_latlon(convert_to_utm(float(datalist[2])), convert_to_utm(float(datalist[4]))))
                # print(position_utm)
                msg.altitude = float(datalist[9])
                msg.utc = float(datalist[1])
                msg.utm_easting, msg.utm_northing, msg.zone, msg.zone_letter = utm.from_latlon(convert_to_utm(float(datalist[2])), convert_to_utm(-float(datalist[4])))
                pub.publish(msg)
                print("published")
                rate.sleep()


if __name__ == "__main__":
    gps_processor()