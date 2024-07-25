import rospy
from sensor_msgs.msg import Image
from ultralytics import YOLO


perception_rgb_publisher = rospy.Publisher('/rgb_perception', Image, queue_size=9)
perception_depth_publisher = rospy.Publisher('/depth_perception', Image, queue_size=9)


def rgb_callback(msg):
    global perception_publisher
    perception_rgb_publisher.publish(msg)
    rospy.loginfo("get and publish rgb message")

def depth_callback(msg):
    global perception_depth_publisher
    perception_depth_publisher.publish(msg)
    rospy.loginfo("get and publish depth message")

 


def sensor_perception():
    rospy.init_node('airsim_image_subscriber', anonymous=True)
    
    # rgb image in camera_1
    vehicle_name = rospy.get_param("/vehicle_name", "drone_1")
    rgb_camera = "camera_1"
    camera_type_scene = "Scene"
    rgb_topic = "/airsim_node/{}/{}/{}".format(vehicle_name, rgb_camera, camera_type_scene)
    rospy.Subscriber(rgb_topic, Image, rgb_callback)
    rospy.loginfo("Subscribed to topic: {}".format(rgb_topic))

    # depth image in camera_2
    depth_camera = "camera_2"
    camera_type_depth = "DepthVis"
    depth_topic = "/airsim_node/{}/{}/{}".format(vehicle_name, depth_camera, camera_type_depth)
    rospy.Subscriber(depth_topic,Image,depth_callback)
    rospy.loginfo(f"Subscribed to topic:{depth_topic}")

    rospy.spin()

if __name__ == "__main__":
    sensor_perception()
