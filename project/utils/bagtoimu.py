import os
import argparse
import numpy as np
import pandas as pd

import rosbag
import matplotlib.pyplot as plt


def main():
    gps = np.zeros((182,2))
    bag = rosbag.Bag('consif23.bag', "r")

    i = 0
    for topic, msg, t in bag.read_messages(topics='/imu/imu_compensated'):
        # print(msg)
        # print(msg.orientation)
        # print(msg.linear_acceleration)
        # print(msg.angular_velocity)
        if i ==0:
            imu_readings = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w, msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z, msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
            imu = [imu_readings]
            i += 1
            continue
        imu_readings = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w, msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z, msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
        imu = np.vstack((imu,[imu_readings]))
        print(i)
        i += 1

    pd1 = pd.DataFrame(imu)
    pd1.to_csv('imu.csv',index=False,header=False)
    bag.close()

    return

if __name__ == '__main__':
    main()