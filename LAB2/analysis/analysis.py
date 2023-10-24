import numpy as np
import matplotlib.pyplot as plt
import rospy
import rosbag
import pandas as pd
import utm
from math import radians, sin, cos, sqrt, asin
import argparse
import statistics

#Analysis on RTK GPS data from ROS bags

#Read in data from ROS bag
def haversine(lat1, lon1, lat2, lon2):
    R = 6372.8  # Earth radius in kilometers

    dLat = radians(lat2 - lat1)
    dLon = radians(lon2 - lon1)
    lat1 = radians(lat1)
    lat2 = radians(lat2)

    a = sin(dLat / 2)**2 + cos(lat1) * cos(lat2) * sin(dLon / 2)**2
    c = 2 * asin(sqrt(a))

    return abs((R * c)*100000) #in cm

def convert_bag_files_to_csv(path_to_bag_file):
    """Helper function to convert bag files to csv files.

    Args:
        path_to_bag_file (string): Path to the bag file.
    """
    bag = rosbag.Bag(path_to_bag_file)
    data = []
    for topic, msg, t in bag.read_messages(topics=["/gps"]):
        # print(msg)
        data.append({
            "time": msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9,
            "latitude": msg.latitude,
            "longitude": msg.longitude,
            "altitude": msg.altitude,
            "utm_easting": msg.utm_easting,
            "utm_northing": msg.utm_northing,
            "zone": msg.zone,
            "zone_letter": msg.letter
        })

    df = pd.DataFrame(data)
    # filename = os.path.basename(path_to_bag_file)
    # filename = filename.split(".")[0]
    # data_dir = os.path.dirname(path_to_bag_file)
    # df.to_csv(os.path.join(data_dir, filename+".csv"), index=False)
    # print("Converting bag file to csv file is done.")
    bag.close()
    print(df.head())
    return df


def calculate_errors(df, true_latitude, true_longitude):
    #calculate haversine error
    errors = []
    for i in range(len(df["latitude"])-1):
        errors.append(haversine(df["latitude"][i], df["longitude"][i], true_latitude, true_longitude))

    return errors

def convert_to_utm(data):
    data = data / 100
    degrees = int(data)
    minutes = (data - degrees) * 100  # Correcting the calculation here
    utm_data = degrees + minutes / 60
    return utm_data

def plot_data(df, args):

    if args.stationary:
        true_latitude =  42.33862
        true_longitude =  -71.08575
        #Plot easting and northing
        #scale easting and northing values

        plt.figure()

        true_easting, true_northing, _, _ = utm.from_latlon(convert_to_utm(float(true_latitude*100)), convert_to_utm(float(true_longitude*100)))  #Apparently the conversion this function gives is not that accurate
        print(true_easting, true_northing)
        true_easting = 328178.38  #Using True values from the internet, which are more accurate.
        true_northing =  4689480.78
        print(true_easting, true_northing)

        easting_std_dev = statistics.stdev(df["utm_easting"])
        northing_std_dev = statistics.stdev(df["utm_northing"])
        labels = ['Easting', 'Northing']
        values = [easting_std_dev, northing_std_dev]
        plt.bar(labels, values)

        plt.xlabel('Coordinate Type')
        plt.ylabel('Standard Deviation')
        plt.title('Standard Deviation of Easting and Northing Values')

        plt.show()

        df["utm_easting"] = df["utm_easting"] - true_easting
        df["utm_northing"] = df["utm_northing"] - true_northing
        plt.scatter(df["utm_easting"], df["utm_northing"], s=20, marker = 'o')
        plt.scatter(0,0, s=40, marker = '*')


        # df["utm_easting"] = df["utm_easting"] - df["utm_easting"][0]
        # df["utm_northing"] = df["utm_northing"] - df["utm_northing"][0]
        # plt.scatter(df["utm_easting"], df["utm_northing"], s=20, marker = 'o')

        plt.xlabel("Easting")
        plt.ylabel("Northing")
        plt.title("Easting vs Northing")
        # plt.savefig("Easting vs Northing.png")
        plt.show()


        #Plot altitude
        plt.figure()
        time_array = np.array(df["time"] - df["time"][0])
        plt.scatter(time_array, df["altitude"], s=20, marker = '*')
        plt.xlabel("Time")
        plt.ylabel("Altitude")
        plt.title("Altitude vs Time")
        # plt.savefig("Altitude vs Time.png")
        plt.show()


        #plot latitude longitude error histogram
        plt.figure()
        errors = calculate_errors(df, true_latitude, true_longitude)
        plt.hist(errors, bins=100, alpha=0.5, color='b', edgecolor='black')
        plt.xlabel("Latitude Longitude Error (cm)")
        plt.ylabel("Frequency")
        plt.title("Latitude Longitude Error Histogram for Stationary Case")
        # plt.savefig("Latitude Longitude Error Histogram.png")
        plt.show()

    elif args.occluded:
        true_latitude = 42.33879
        true_longitude = -71.0883
        plt.figure()
        true_easting, true_northing, _, _ = utm.from_latlon(convert_to_utm(float(true_latitude*100)), convert_to_utm(float(true_longitude*100)))
        true_easting = 327968.77
        true_northing = 4689504.81
        print(true_easting, true_northing)
        easting_std_dev = statistics.stdev(df["utm_easting"])
        northing_std_dev = statistics.stdev(df["utm_northing"])
        labels = ['Easting', 'Northing']
        values = [easting_std_dev, northing_std_dev]
        plt.bar(labels, values)

        plt.xlabel('Coordinate Type')
        plt.ylabel('Standard Deviation')
        plt.title('Standard Deviation of Easting and Northing Values')

        plt.show()

        df["utm_easting"] = df["utm_easting"] - true_easting
        df["utm_northing"] = df["utm_northing"] - true_northing
        plt.scatter(0,0, s=40, marker = '*')
        plt.scatter(df["utm_easting"], df["utm_northing"], s=20, marker = 'o')

        # df["utm_easting"] = df["utm_easting"] - df["utm_easting"][0]
        # df["utm_northing"] = df["utm_northing"] - df["utm_northing"][0]
        # plt.scatter(df["utm_easting"], df["utm_northing"], s=20, marker = 'o')

        plt.xlabel("Easting")
        plt.ylabel("Northing")
        plt.title("Easting vs Northing")
        # plt.savefig("Easting vs Northing.png")
        plt.show()

        #Plot altitude
        plt.figure()
        time_array = np.array(df["time"] - df["time"][0])
        plt.scatter(time_array, df["altitude"], s=20, marker = '*')
        plt.xlabel("Time")
        plt.ylabel("Altitude")
        plt.title("Altitude vs Time")
        # plt.savefig("Altitude vs Time.png")
        plt.show()
        #plot latitude longitude error histogram
        plt.figure()
        errors = calculate_errors(df, true_latitude, true_longitude)
        plt.hist(errors, bins=100, alpha=0.5, color='b', edgecolor='black')
        plt.xlabel("Latitude Longitude Error (cm)")
        plt.ylabel("Frequency")
        plt.title("Latitude Longitude Error Histogram for Stationary and Occluded Case")
        # plt.savefig("Latitude Longitude Error Histogram.png")
        plt.show()

    else:
        plt.figure()
        easting_std_dev = statistics.stdev(df["utm_easting"])
        northing_std_dev = statistics.stdev(df["utm_northing"])
        labels = ['Easting', 'Northing']
        values = [easting_std_dev, northing_std_dev]
        plt.bar(labels, values)

        plt.xlabel('Coordinate Type')
        plt.ylabel('Standard Deviation')
        plt.title('Standard Deviation of Easting and Northing Values for Open Case')

        plt.show()
        df["utm_easting"] = df["utm_easting"] - df["utm_easting"][0]
        df["utm_northing"] = df["utm_northing"] - df["utm_northing"][0]
        plt.scatter(df["utm_easting"], df["utm_northing"], s=20, marker = 'o')
        plt.xlabel("Easting")
        plt.ylabel("Northing")
        plt.title("Easting vs Northing")
        # plt.savefig("Easting vs Northing.png")
        plt.show()

        #Plot altitude
        plt.figure()
        time_array = np.array(df["time"] - df["time"][0])
        plt.scatter(time_array, df["altitude"], s=20, marker = '*')
        plt.xlabel("Time")
        plt.ylabel("Altitude")
        plt.title("Altitude vs Time")
        # plt.savefig("Altitude vs Time.png")
        plt.show()

if __name__ == "__main__":
    path = 'Fw_ lab2 data/2023-10-16-15-00-42.bag'
    parser = argparse.ArgumentParser(description="Plot the data from the bag file.")
    parser.add_argument("--path", help="Path to the bag file.", default= "/home/tarun/catkin_ws/src/Data/2023-10-07-13-54-33.bag")
    parser.add_argument("--stationary", help="Plot the data for stationary case.", action="store_true")
    parser.add_argument("--occluded", help="Plot the data for stationary and occluded case.", action="store_true")
    parser.add_argument("--moving", help="Plot the data for moving case.", action="store_true")
    args = parser.parse_args()
    df = convert_bag_files_to_csv(args.path)
    plot_data(df ,args)