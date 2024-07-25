import rospy
from cv_bridge import CvBridge,CvBridgeError
import cv2
from sensor_msgs.msg import Image
from ultralytics import YOLO
import ultralytics
import copy

# read yolo
model = YOLO("/home/lenovo/myws/yolov8_train/src/yolov8n_v1.pt")
annotated_frame_publisher = rospy.Publisher('/annotated_image', Image, queue_size=9)
uv_coord_publisher = rospy.Publisher("/2Dpoints")

def get_annotated_frame(result:ultralytics.engine.results.Results):
    result_temp = copy.deepcopy(result)
    label_key = str(result.boxes.cls)
    xyxy = result_temp.boxes.xyxy
    first_box = xyxy[0]
    x1, y1, x2, y2 = first_box[0], first_box[1], first_box[2], first_box[3]
    x1 = int(round(x1.item()))
    y1 = int(round(y1.item()))
    x2 = int(round(x2.item()))
    y2 = int(round(y2.item()))
    temp = cv2.rectangle(result_temp.orig_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
    temp = cv2.putText(temp, label_key, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 2)
    return temp

def rgb_track_callback(msg):
    bridge = CvBridge()
    try:
        # Convert the ROS Image message to a format OpenCV can work with
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        result = model.track(cv_image)[0]
        

        annotated_image=get_annotated_frame(result) 

        # Convert the processed image (result) back to a ROS Image message
        annotated_msg = bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8')

        # Publish the result
        annotated_frame_publisher.publish(annotated_msg)

    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))

    # Display the image
    # cv2.imshow("AirSim Camera Image", cv_image)
    # cv2.waitKey(10)
    # rospy.loginfo("get rgb")

def depth_callback(msg):
    bridge = CvBridge()
    try:
        # Convert the ROS Image message to a format OpenCV can work with
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))

    # Display the image
    cv2.imshow("AirSim Camera Image", cv_image)
    cv2.waitKey(10)
    rospy.loginfo("get depth")

# def test_callback(msg):
#     rospy.loginfo("hello")

def yolo_image_prcessor():
    rospy.init_node('image_processor', anonymous=True)
    rospy.Subscriber('/rgb_perception',Image,rgb_track_callback)
    rospy.Subscriber("/rgb_perception",Image,)

    # # depth image in camera_2
    # depth_camera = "camera_2"
    # camera_type_depth = "DepthVis"
    # depth_topic = "/airsim_node/{}/{}/{}".format(vehicle_name, depth_camera, camera_type_depth)
    # test_topic = "/airsim_node/drone_1/camera_2/DepthVis"
    # rospy.Subscriber(test_topic,Image,test_callback)
    # rospy.loginfo(f"Subscribed to topic:{depth_topic}")
    rospy.spin()

if __name__ == "__main__":
   yolo_image_prcessor()