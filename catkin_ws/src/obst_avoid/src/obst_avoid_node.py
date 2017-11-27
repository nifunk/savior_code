#!/usr/bin/env python
import rospy
from sensor_msgs.msg import CompressedImage

### note you need to change the name of the robot to yours here
from obst_avoid.detector import Detector
from duckietown_utils import get_base_name, rgb_from_ros

class ObstDetectNode(object):
    """
    Obstacle Detection Node
    """
    def __init__(self):
        self.node_name = "Obstacle Detecion Node"
        robot_name = rospy.get_param("~robot_name", "")

        self.detector = Detector(robot_name=robot_name)

        # Create a Publisher
        self.pub_topic = '/{}/ObstDetect/image/compressed'.format(robot_name)
        self.publisher = rospy.Publisher(self.pub_topic, CompressedImage, queue_size=1)

        # Create a Subscriber
        self.sub_topic = '/{}/camera_node/image/compressed'.format(robot_name)
        self.subscriber = rospy.Subscriber(self.sub_topic, CompressedImage, self.callback)

    def callback(self, image):
	#print "HELLO CALLLBACK HERE"
        obst_image = CompressedImage()
        obst_image.header.stamp = image.header.stamp
        obst_image.format = "jpeg"

        # you should write the following function in your class
        obst_image.data = self.detector.process_image(rgb_from_ros(image))

	#spaeter: anstatt obst_image.data eher
	#1. EXTRACT OBSTACLES 
	#2. SEND THEM
	#3. VISUALIE THEM!

        # publish new message
        self.publisher.publish(obst_image.data)

    def onShutdown(self):
        rospy.loginfo('Shutting down Obstacle Detection, back to unsafe mode')

# MEINER MEINUNG NACH HIER DANN WARSCH 2.NODE AUCH NOCH REIN WO DANN DIE OBST AVOIDANCE GEMACHT WIRD ODER SO

if __name__ == '__main__':
    rospy.init_node('obst_detection_node', anonymous=False)
    obst_detection_node = ObstDetectNode()
    rospy.on_shutdown(obst_detection_node.onShutdown)
    rospy.spin()
