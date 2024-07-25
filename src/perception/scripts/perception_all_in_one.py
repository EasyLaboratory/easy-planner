import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError
from easyGL.airsim_gl import *
from pathlib import Path
from ultralytics import YOLO
from message_filters import Subscriber, TimeSynchronizer, ApproximateTimeSynchronizer
from nav_msgs.msg import Odometry

current_dir = Path(__file__).resolve()
project_root = current_dir.parent
model_base_path = project_root/"model"
model_path = model_base_path/"yolov8n_v1.pt"

model:ultralytics.YOLO = YOLO(model_path)
rospy.loginfo("init the yolo model")

bridge = CvBridge()
annotated_frame_publisher = rospy.Publisher("/annotated_image",Image,queue_size=9)
odom_publisher = rospy.Publisher('/target/odom_airsim', Odometry, queue_size=10)


def perception_callback(rgb_msg,depth_msg):
    global annotated_frame_publisher
    global bridge
    global model
    
    try:
        # Convert the ROS Image message to a format OpenCV can work with
        cv_image = bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='passthrough')
        result=track(model,cv_image)[0]
        boxes,category,id = get_target_box(result,[0,1,2,3,4,5,6,7,8,9,10,11],[1])
        rospy.loginfo(boxes)
        uv_list = get_uv(boxes)
        rospy.loginfo(uv_list)
        annotated_image=get_annotated_frame(result,boxes,uv_list) 
        # Convert the processed image (result) back to a ROS Image message
        annotated_msg = bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8')

        # Publish the annotated
        annotated_frame_publisher.publish(annotated_msg)
        
           # 创建一个新的Odometry消息
        odom_msg = Odometry()
        
        # 填充消息内容
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = "odom_frame"
        odom_msg.child_frame_id = "base_link"
        
        # 填充pose部分
        odom_msg.pose.pose.position.x = 1.0
        odom_msg.pose.pose.position.y = 2.0
        odom_msg.pose.pose.position.z = 3.0
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = 0.0
        odom_msg.pose.pose.orientation.w = 1.0
        
        # 填充twist部分
        odom_msg.twist.twist.linear.x = 0.1
        odom_msg.twist.twist.linear.y = 0.2
        odom_msg.twist.twist.linear.z = 0.3
        odom_msg.twist.twist.angular.x = 0.01
        odom_msg.twist.twist.angular.y = 0.02
        odom_msg.twist.twist.angular.z = 0.03
        rospy.loginfo(odom_msg)
        rospy.loginfo("-----------------------------------")
        odom_publisher.publish(odom_msg)
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))

    rospy.loginfo("get and publish rgb message")

def sensor_perception():
    rospy.init_node('airsim_image_subscriber', anonymous=True)
    
    # rgb image in camera_1
    vehicle_name = rospy.get_param("/vehicle_name", "drone_1")
    rgb_camera = "camera_1"
    camera_type_scene = "Scene"
    rgb_topic = "/airsim_node/{}/{}/{}".format(vehicle_name, rgb_camera, camera_type_scene)

    # depth image in camera_2
    depth_camera = "camera_2"
    camera_type_depth = "DepthVis"
    depth_topic = "/airsim_node/{}/{}/{}".format(vehicle_name, depth_camera, camera_type_depth)

    rgb_sub = Subscriber(rgb_topic,Image)
    depth_sub = Subscriber(depth_topic,Image)

    ats = ApproximateTimeSynchronizer([rgb_sub, depth_sub], queue_size=10, slop=0.1)
    ats.registerCallback(perception_callback)
    rospy.spin()

if __name__ == "__main__":
    sensor_perception()