import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import rospy
import argparse
import rosbag
from squaternion import Quaternion
from scipy.integrate import cumtrapz

def calibrate_magnetometer(data_samples):
    # Extract x, y, z data
    x = data_samples[:, 0]*10**6
    y = data_samples[:, 1]*10**6

    # Calculate hard iron offset (average of mforward_velocity_imu and min for each forward_velocity_imuis)
    hard_iron_offset = np.array([(np.max(x) + np.min(x)) / 2.0,
                                 (np.max(y) + np.min(y)) / 2.0])

    # Remove hard iron offset
    mag_hard_removed_x = x - hard_iron_offset[0]
    mag_hard_removed_y = y - hard_iron_offset[1]

    magxy = np.zeros((2,len(x)))
    magxy[0] = mag_hard_removed_x
    magxy[1] = mag_hard_removed_y


    r = np.sqrt((mag_hard_removed_x)**2+(mag_hard_removed_y)**2)
    m_v,m_i = min(r), np.argmin(r)
    r_v,r_i = max(r), np.argmax(r)

    plt.figure(figsize=(10,10))
    plt.plot(mag_hard_removed_x,mag_hard_removed_y)
    plt.scatter(mag_hard_removed_x[r_i],mag_hard_removed_y[r_i], c='r', label = "Mforward_velocity_imu")
    plt.scatter(mag_hard_removed_x[m_i],mag_hard_removed_y[m_i], c='g', label = "Min")
    plt.scatter(0,0, c='k', label = "Origin")
    plt.title("Magnetometer Hard/Soft calibration")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.legend()
    plt.show()

    theta = np.arcsin(mag_hard_removed_y[r_i]/r_v)
    R = np.array([[np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)]])
    magxy_soft_iron = np.matmul(R,magxy)

    scale_factor = m_v/r_v
    magxy_soft_iron= magxy_soft_iron* scale_factor
    theta = -theta
    R = np.array([[np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)]])
    magxy_soft_iron = np.matmul(R,magxy_soft_iron)

    plt.figure(figsize=(10, 10))
    plt.plot(x, y, label="Raw Data")
    plt.plot(magxy_soft_iron[0], magxy_soft_iron[1], label="Calibrated Data")
    plt.legend()
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("Calibrated Magnetometer Data")
    plt.show()

    return hard_iron_offset, R, scale_factor

def calibrate_magnetometer_with_ellipsoid(data_samples, hard_iron_offset, soft_iron_transformation, scale_factor):
    x = data_samples[:, 0]*10**6
    y = data_samples[:, 1]*10**6

    real_magx = x - hard_iron_offset[0]
    real_magy = y - hard_iron_offset[1]
    # remove soft iron

    real_magxy = np.zeros((2,len(real_magx)))
    real_magxy[0] = real_magx
    real_magxy[1] = real_magy
    _ = np.matmul(soft_iron_transformation,real_magxy)

    real_magxy= real_magxy * scale_factor
    calibrated_xy = np.matmul(-(soft_iron_transformation),real_magxy)

    return calibrated_xy


def estimate_yaw(calibrated_xy, imu_orientation):

    yaw_raw = np.arctan2(calibrated_xy[1],calibrated_xy[0])* 180 / np.pi
    euler_angle = np.zeros((len(imu_orientation),3))
    for i in range(len(imu_orientation)):
        w = imu_orientation[i][3]
        x = imu_orientation[i][0]
        y = imu_orientation[i][1]
        z = imu_orientation[i][2]
        q = Quaternion(w,x,y,z)
        e = q.to_euler()
        euler_angle[i,:] = e
    yaw_angles = euler_angle[:,2]* 180 / np.pi

    return yaw_raw , yaw_angles

def plot_yaw_angles(yaw_angles_raw, yaw_angles_calibrated):
    plt.figure(figsize=(20, 10))
    plt.plot(yaw_angles_raw, label="Magnetometer Yaw")
    plt.plot(yaw_angles_calibrated, label="IMU Yaw")
    plt.legend()
    plt.xlabel("Time (s)")
    plt.ylabel("Yaw Angle (degrees)")
    plt.title("Yaw Angle vs Time")
    plt.show()

def estimate_yaw_rate(yaw_angles_calibrated, imu_angular_velocity, time_data):
    yaw_rate = imu_angular_velocity[:,2]

    yaw_angle = cumtrapz(yaw_rate, time_data)*180/np.pi
    yaw_angle = yaw_angle + yaw_angles_calibrated[0]
    angle =  yaw_angle % 360
    angle = (angle + 360) % 360


    for i in range(len(yaw_angle)):
        if angle[i] > 180:
            angle[i] -= 360

    yaw_imu = angle
    return yaw_rate, yaw_imu

def plot_yaw_comparison_imu(yaw_angles_imu, yaw_angles_calibrated):
    plt.figure(figsize=(20, 10))
    plt.plot(yaw_angles_imu, label="IMU Data")
    plt.plot(yaw_angles_calibrated, label="Calibrated Mag Data")
    plt.legend()
    plt.xlabel("Time (s)")
    plt.ylabel("Yaw Angle (degrees)")
    plt.title("Yaw Angle vs Time")
    plt.show()

def complimentary_filter(yaw_angles_imu, yaw_angles_mag):
    alpha = 0.9

    yaw_angles_comp_filter = alpha * yaw_angles_imu + (1 - alpha) * yaw_angles_mag[1:]

    return yaw_angles_comp_filter

def plot_yaw_comparison_final(yaw_angles_comp_filter, yaw_angles_calibrated, yaw_angles_imu):
    plt.figure(figsize=(20, 10))
    plt.plot(yaw_angles_comp_filter, label="Complimentary Filter")
    plt.plot(yaw_angles_calibrated*0.1, label="Calibrated Mag Data with low-pass")
    plt.plot(yaw_angles_imu*0.9, label="IMU Data with high-pass")
    plt.legend()
    plt.xlabel("Time (s)")
    plt.ylabel("Yaw Angle (degrees)")
    plt.title("Yaw Angle vs Time")
    plt.show()

def calculate_forward_velocity(location_data, time_data_gps):
    # Calculate forward velocity
    forward_velocity = np.zeros(len(location_data))
    gps_vx = np.zeros((len(location_data)))
    gps_vy = np.zeros((len(location_data)))
    for i in range(len(location_data) - 1):
        forward_velocity[i] = np.sqrt((location_data[i + 1][0] - location_data[i][0]) ** 2 + (
                    location_data[i + 1][1] - location_data[i][1]) ** 2) / (time_data_gps[i + 1] - time_data_gps[i])

        gps_vx[i] = (location_data[i + 1][0] - location_data[i][0]) / (time_data_gps[i + 1] - time_data_gps[i])
        gps_vy[i] = (location_data[i + 1][1] - location_data[i][1]) / (time_data_gps[i + 1] - time_data_gps[i])

    return forward_velocity , gps_vx, gps_vy

def plot_forward_velocity(forward_velocity):
    plt.figure(figsize=(20, 10))
    plt.plot(forward_velocity)
    plt.xlabel("Time (s)")
    plt.ylabel("Forward Velocity (m/s)")
    plt.title("Forward Velocity from GPS vs Time")
    plt.show()

def calculate_forward_velocity_imu(linear_acceleration, imu_t):
    linear_acceleration = np.array(linear_acceleration)
    ax = linear_acceleration[:,0]
    ay = linear_acceleration[:,1]
    az = linear_acceleration[:,2]

    forward_velocity =  np.zeros((len(ax)))
    forward_velocity[1150:4000-1] = cumtrapz(ax[1150:4000]-np.mean(ax[1150:4000]),imu_t[1150:4000])
    forward_velocity[4000:11500-1] = cumtrapz(ax[4000:11500]-np.mean(ax[4000:11500]),imu_t[4000:11500])
    forward_velocity[11500:15000-1] = cumtrapz(ax[11500:15000]-np.mean(ax[11500:15000]),imu_t[11500:15000])
    # forward_velocity[16000:18000-1] = cumtrapz(ax[16000:18000]-np.mean(ax[16000:18000]),imu_t[16000:18000])

    # forward_velocity[4600:15000-1] = cumtrapz(ax[4600:15000]-np.mean(ax[4600:15000]),imu_t[4600:15000])
    forward_velocity[15000:18000-1] = cumtrapz(ax[15000:18000]-np.mean(ax[15000:18000]),imu_t[15000:18000])
    forward_velocity[18000:22800-1] = cumtrapz(ax[18000:22800]-np.mean(ax[18000:22800]),imu_t[18000:22800])
    forward_velocity[24000:31000-1] = cumtrapz(ax[24000:31000]-np.mean(ax[24000:31000]),imu_t[24000:31000])
    forward_velocity[31500:44500-1] = cumtrapz(ax[31500:44500]-np.mean(ax[31500:44500]),imu_t[31500:44500])

    return ax, ay, forward_velocity

def plot_forward_velocity_comparison(forward_velocity, ax, forward_velocity_imu, imu_t, gps_t):
    plt.figure(figsize=(20, 10))
    delta_t = imu_t[0] - gps_t[0]
    gps_t = gps_t + delta_t
    plt.plot(imu_t, ax, label="IMU Linear Acceleration (x-axis)")
    plt.plot(gps_t, forward_velocity, label="GPS Forward Velocity")
    plt.legend()
    plt.xlabel("Time (s)")
    plt.ylabel("Forward Velocity (m/s) / Linear Acceleration (m/s^2)")
    plt.title("Forward Velocity vs Time")
    plt.show()

    plt.figure(figsize=(20, 10))
    plt.plot(gps_t, forward_velocity, label="GPS Forward Velocity")
    plt.plot(imu_t[1150:4000],ax[1150:4000])
    plt.plot(imu_t[4000:11500],ax[4000:11500])
    plt.plot(imu_t[11500:15000],ax[11500:15000])
    # plt.plot(imu_t[16000:18000],ax[16000:18000])
    # plt.plot(imu_t[4600:15000],ax[4600:15000])
    plt.plot(imu_t[15000:18000],ax[15000:18000])
    plt.plot(imu_t[18300:22800],ax[18300:22800])
    plt.plot(imu_t[24000:31000],ax[24000:31000])
    plt.plot(imu_t[31500:44500],ax[31500:44500])
    plt.title("Splitting the data into sections")
    plt.legend()
    plt.show()

    plt.figure(figsize=(20, 10))
    plt.plot(imu_t, forward_velocity_imu, label="IMU Forward Velocity")
    plt.plot(gps_t, forward_velocity, label="GPS Forward Velocity")
    plt.legend()
    plt.xlabel("Time (s)")
    plt.ylabel("Forward Velocity (m/s)")
    plt.title("Forward Velocity vs Time")
    plt.show()


def dead_reckoning_imu(yaw_angles_imu, forward_velocity_imu, imu_t, imu_n_t, gps_t, yaw_rate, ay , gps_vx, gps_vy, gps_loc):

    w = yaw_rate
    v = forward_velocity_imu
    y_dot = w*v
    plt.figure(figsize=(20,10))
    plt.plot(ay, label = "wX")
    plt.plot(y_dot,c='r', label = "df(df(ay))")
    plt.title("xW vs df(df(y))")
    plt.legend()
    plt.show()

    v_e = v[0:len(yaw_angles_imu)]*np.cos(yaw_angles_imu*np.pi/180)
    v_n = v[0:len(yaw_angles_imu)]*np.sin(yaw_angles_imu*np.pi/180)

    delta_t = imu_t[0] - gps_t[0]
    gps_t = gps_t + delta_t
    plt.figure(figsize=(20,10))
    plt.plot(imu_t[0:len(yaw_angles_imu)],v_n)
    plt.plot(gps_t,gps_vx, label = "GPS")
    plt.title("Velocities_Vn vs GPS values")
    plt.legend()
    plt.show()

    plt.figure(figsize=(20,10))
    plt.plot(imu_t[0:len(yaw_angles_imu)],v_e)
    plt.plot(gps_t,gps_vy, label = "GPS")
    plt.title("Velocities_Ve vs GPS values")
    plt.legend()
    plt.show()

    actual_time_0 = imu_t[0] + imu_n_t[0]*10**-7
    actual_time_1 = imu_t[1] + imu_n_t[1]*10**-7
    dt = actual_time_1 - actual_time_0
    # print(dt)
    x_e = [0]
    x_n = [0]
    for i in range(len(v_e)):
        dx_e = v_e[i]*dt
        dx_n = v_n[i]*dt
        x_e.append(x_e[i]+dx_e)
        x_n.append(x_n[i]+dx_n)

    th = 45 * np.pi/180
    R = [[np.cos(th), -np.sin(th)],[np.sin(th),np.cos(th)]]
    R = np.array(R)
    scale = 0.85
    imu_trac = np.zeros((2,len(x_e)))
    imu_trac[0] = np.array(x_n)
    imu_trac[1] = np.array(x_e)
    imu_final = np.matmul(R,imu_trac)*scale
    plt.figure(figsize=(20,10))
    plt.plot(x_n,x_e)
    plt.title("Dead Reckoning")
    plt.show()

    gps_loc = np.array(gps_loc)
    utm_east = gps_loc[:,0]
    utm_north = gps_loc[:,1]
    # print(utm_east[0])

    plt.figure(figsize=(20,10))
    plt.plot(utm_east-utm_east[0],utm_north-utm_north[0], label = "GPS")
    plt.plot(imu_final[0]+80,imu_final[1]+370, label = "IMU")
    plt.title("Dead Reckoning comp/ GPS")
    plt.legend()
    plt.show()


    xc = []
    x = ay-y_dot
    ang_acc = forward_velocity_imu[1:len(forward_velocity_imu)]-forward_velocity_imu[0:len(forward_velocity_imu)-1]
    ang_acc = ang_acc*40
    for i in range(len(ang_acc)):
        if ang_acc[i] != 0:
            xc.append(x[i]/ang_acc[i])
    x_c = np.mean(xc)

    print("Xc = ", x_c)



def extract_messages(rosbag):
    # Extract data from ROS bag
    magfield_data = []
    orientation_data = []
    angular_velocity_data = []
    location_data = []
    time_data = []
    time_data_gps = []
    linear_acceleration_data = []
    nano_time_data = []

    for topic, msg, t in rosbag.read_messages(topics=["/IMU"]):
        magfield_data.append([msg.MagField.magnetic_field.x,
                            msg.MagField.magnetic_field.y,
                            msg.MagField.magnetic_field.z])

        orientation_data.append([msg.IMU.orientation.x,
                            msg.IMU.orientation.y,
                            msg.IMU.orientation.z,
                            msg.IMU.orientation.w])

        angular_velocity_data.append([msg.IMU.angular_velocity.x,
                                msg.IMU.angular_velocity.y,
                                msg.IMU.angular_velocity.z])

        linear_acceleration_data.append([msg.IMU.linear_acceleration.x,
                                msg.IMU.linear_acceleration.y,
                                msg.IMU.linear_acceleration.z])

        time_data.append(msg.Header.stamp.secs)

        nano_time_data.append(msg.Header.stamp.nsecs)

    for topic,msg,t in rosbag.read_messages(topics=["/gps"]):
        time_data_gps.append(msg.Header.stamp.secs)
        location_data.append([msg.UTM_easting,msg.UTM_northing])

    magfield_data = np.array(magfield_data)
    orientation_data = np.array(orientation_data)
    angular_velocity_data = np.array(angular_velocity_data)
    time_data = np.array(time_data)
    location_data = np.array(location_data)
    time_data_gps = np.array(time_data_gps)

    return magfield_data, orientation_data, angular_velocity_data, linear_acceleration_data, time_data , nano_time_data, location_data, time_data_gps

def handle_ros_bags_call_functions(args):

    circle_magfield_data, circle_orientation_data, circle_angular_velocity,circle_linear_acc, circle_time,_, _, _ = extract_messages(rosbag.Bag(args.circle_bag))
    tour_magfield_data, tour_orientation_data, tour_angular_velocity, tour_linear_acc, tour_time ,tour_nano_time, tour_loc_gps, tour_time_gps = extract_messages(rosbag.Bag(args.tour_bag))

    circle_magfield_data = np.array(circle_magfield_data)
    tour_magfield_data = np.array(tour_magfield_data)

    # Perform magnetometer calibration using circle data
    hard_iron_offset, soft_iron_transformation, scale_factor = calibrate_magnetometer(circle_magfield_data)

    # Perform magnetometer calibration with ellipsoid
    calibrated_tour_data = calibrate_magnetometer_with_ellipsoid(tour_magfield_data, hard_iron_offset, soft_iron_transformation, scale_factor)

    yaw_angles_mag, yaw_angles_calibrated = estimate_yaw(calibrated_tour_data, tour_orientation_data)
    plot_yaw_angles(yaw_angles_mag, yaw_angles_calibrated)

    yaw_rate, yaw_angles_imu = estimate_yaw_rate(yaw_angles_calibrated, tour_angular_velocity, tour_time)
    plot_yaw_comparison_imu(yaw_angles_imu, yaw_angles_calibrated)

    yaw_angles_comp_filter = complimentary_filter(yaw_angles_imu, yaw_angles_mag)
    plot_yaw_comparison_final(yaw_angles_comp_filter, yaw_angles_calibrated, yaw_angles_imu)

    forward_velocity, gps_vx, gps_vy = calculate_forward_velocity(tour_loc_gps, tour_time_gps)
    plot_forward_velocity(forward_velocity)

    ax, ay, forward_velocity_imu = calculate_forward_velocity_imu(tour_linear_acc, tour_time)
    plot_forward_velocity_comparison(forward_velocity, ax, forward_velocity_imu ,tour_time, tour_time_gps)

    dead_reckoning_imu(yaw_angles_imu, forward_velocity_imu, tour_time, tour_nano_time, tour_time_gps, yaw_rate, ay, gps_vx, gps_vy, tour_loc_gps)


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="LAB4 Analysis")
    parser.add_argument("--circle_bag", type=str, help="Path to the ROS bag", required=True)
    parser.add_argument("--tour_bag", type=str, help="Path to the ROS bag", required = True)
    args = parser.parse_args()
    handle_ros_bags_call_functions(args)