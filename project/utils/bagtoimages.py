import os
import argparse

import cv2
import time
import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def main():

    bag = rosbag.Bag('consif23.bag', 'r')
    bridge = CvBridge()
    count = 0

    os.makedirs('./bagtoimg/project_data/image_0')
    os.makedirs('./bagtoimg/project_data/image_1')
    os.makedirs('./bagtoimg/project_data/image_2')
    os.makedirs('./bagtoimg/project_data/image_3')
    os.makedirs('./bagtoimg/project_data/image_4')

    for topic, msg, t in bag.read_messages(topics='/camera_array/cam0/image_raw'):
        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

        cv2.imwrite(os.path.join('./bagtoimg/project_data/image_0', '0'*(6-len(str(count))) + str(count) + ".png"), cv_img)
        print("Wrote image",count)

        count += 1
    count =0
    for topic, msg, t in bag.read_messages(topics='/camera_array/cam1/image_raw'):
        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

        cv2.imwrite(os.path.join('./bagtoimg/project_data/image_1', '0'*(6-len(str(count))) + str(count) + ".png"), cv_img)
        print("Wrote image",count)

        count += 1
    count =0
    for topic, msg, t in bag.read_messages(topics='/camera_array/cam2/image_raw'):
        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

        cv2.imwrite(os.path.join('./bagtoimg/project_data/image_2', '0'*(6-len(str(count))) + str(count) + ".png"), cv_img)
        print("Wrote image",count)

        count += 1
    count =0
    for topic, msg, t in bag.read_messages(topics='/camera_array/cam3/image_raw'):
        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

        cv2.imwrite(os.path.join('./bagtoimg/project_data/image_3', '0'*(6-len(str(count))) + str(count) + ".png"), cv_img)
        print("Wrote image",count)

        count += 1
    count =0
    for topic, msg, t in bag.read_messages(topics='/camera_array/cam4/image_raw'):
        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

        cv2.imwrite(os.path.join('./bagtoimg/project_data/image_4', '0'*(6-len(str(count))) + str(count) + ".png"), cv_img)
        print("Wrote image",count)

        count += 1

    bag.close()

    return

if __name__ == '__main__':
    main()