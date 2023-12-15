import re
import rospy
import serial
from imu_driver.msg import imu_msg
from imu_driver.srv import convert

def service_client(r, p, y):
    # Wait for the service client /euler_to_quaternion to be running
    rospy.wait_for_service('euler_to_quaternion')

    try:
        # Create a proxy
        converter = rospy.ServiceProxy('euler_to_quaternion', convert)
        response = converter(r, p, y)
        return response.qx, response.qy, response.qz, response.qw
    except rospy.ServiceException as e:
        print("Service call failed %s", e)


def talker():
    pub = rospy.Publisher('IMU', imu_msg, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    message = imu_msg()

    port = rospy.get_param('port')

    serial_data = serial.Serial(port, 115200)

    rate = "$VNWRG,7,40"
    serial_data.write(rate.encode())

    while not rospy.is_shutdown():
        value = serial_data.readline()
        value = value.decode("UTF-8")

        if "VNYMR" in value:
            old_data = value.split(",")
            data = []

            for x in old_data:
                data.append(re.sub(r'\\x[0-9a-fA-F]{2}', '', x))

            utc_time = rospy.rostime.get_time()
            utc_nano = (float(utc_time) - int(utc_time)) * (10**7)

            yaw = float(data[1])
            pitch = float(data[2])
            roll = float(data[3])
            message.MagField.magnetic_field.x = float(data[4])*0.0001
            message.MagField.magnetic_field.y = float(data[5])*0.0001
            message.MagField.magnetic_field.z = float(data[6])*0.0001
            message.IMU.linear_acceleration.x = float(data[7])
            message.IMU.linear_acceleration.y = float(data[8])
            message.IMU.linear_acceleration.z = float(data[9])
            message.IMU.angular_velocity.x = float(data[10])
            message.IMU.angular_velocity.y = float(data[11])

            split_z = list(data[12])
            message.IMU.angular_velocity.z = float("".join(split_z[:len(split_z)-5]))

            message.IMU.orientation.x, message.IMU.orientation.y, message.IMU.orientation.z, message.IMU.orientation.w = service_client(roll, pitch, yaw)

            message.Header.stamp.secs = int(utc_time)
            message.Header.stamp.nsecs = int(utc_nano)
            message.Header.frame_id = "IMU1_Frame"

            message.raw = str(data)
            pub.publish(message)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass