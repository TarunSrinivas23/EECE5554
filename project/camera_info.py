import rosbag
import yaml
import json

def msg2json(msg):
    ''' Convert a ROS message to JSON format'''
    y = yaml.safe_load(str(msg))  # Use safe_load instead of load for security reasons
    return json.dumps(y, indent=4, sort_keys=True)

def main():
    bag = rosbag.Bag('consif23.bag', 'r')
    count = 0

    for topic, msg, t in bag.read_messages(topics='/camera_array/cam0/camera_info'):
        print(msg)
        json_format = msg2json(msg)
        break  # Convert only the first message, remove if you want to convert all

    with open('camera_info_0.json', 'w') as outfile:
        outfile.write(json_format)

    for topic, msg, t in bag.read_messages(topics='/camera_array/cam1/camera_info'):
        print(msg)
        json_format = msg2json(msg)
        break  # Convert only the first message, remove if you want to convert all

    with open('camera_info_1.json', 'w') as outfile:
        outfile.write(json_format)

    bag.close()

if __name__ == '__main__':
    main()