import sys
import rospy
from imu_driver.srv import convert, convertResponse

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


if __name__ == '__main__':
    if len(sys.argv) == 4:
        r = int(sys.argv[1])
        p = int(sys.argv[1])
        y = int(sys.argv[1])
    try:
        print(service_client(r, p, y))
    except rospy.ROSInterruptException:
        pass

