import os
import argparse
import numpy as np
import pandas as pd

import rosbag
import matplotlib.pyplot as plt


def main():
    bag = rosbag.Bag('consif23.bag', "r")
    i = 0
    for topic, msg, t in bag.read_messages(topics='/gps/fix'):
        # print(msg)
        if i ==0:
            gps_readings = [msg.latitude,msg.longitude]
            gps = [gps_readings]
            i += 1
            continue
        lat, log = msg.latitude,msg.longitude
        gps_readings = [lat,log]
        gps = np.vstack((gps,[gps_readings]))
        print(i)
        i += 1

    print(gps)
    pd1 = pd.DataFrame(gps)
    pd1.to_csv('gps.csv',index=False,header=False)
    bag.close()

    return

if __name__ == '__main__':
    main()