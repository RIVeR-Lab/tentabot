#!/usr/bin/env python3

'''
LAST UPDATE: 2022.03.11

AUTHOR: stretch_ros
        Hongyu Li (LHY)
        Neset Unver Akmandor (NUA)

E-MAIL: li.hongyu1@northeastern.edu
        akmandor.n@northeastern.edu

DESCRIPTION: TODO...

REFERENCES:

NUA TODO:
- 
'''

from gazebo_msgs.srv import GetModelState, GetModelStateRequest, GetWorldProperties, GetLinkState, GetLinkStateRequest
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from std_srvs.srv import Empty
import rospy
import time

import tf_conversions

import tf2_ros
import geometry_msgs.msg

rospy.init_node('ground_truth_odometry_publisher')
odom_pub=rospy.Publisher('ground_truth', Odometry, queue_size=10)

rospy.wait_for_service('/gazebo/get_model_state')
get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
get_link_srv = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
get_world_properties = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
unpause_physics = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)

odom=Odometry()
header = Header()
header.frame_id='/ground_truth'
model = GetModelStateRequest()
model.model_name='robot'
models = []

link = GetLinkStateRequest()
link.link_name = 'base_link'
link.reference_frame = 'world'
r = rospy.Rate(100)

pause_timeout = time.time() + 4.0
while time.time() < pause_timeout:
    rospy.logwarn("Waiting %.2f seconds to unpause physics", pause_timeout - time.time())
    time.sleep(1.0)
unpause_physics()

br = tf2_ros.TransformBroadcaster()

while not rospy.is_shutdown():
    if model.model_name not in models:
        models = get_world_properties().model_names
        rospy.logwarn("Waiting for %s to spawn to publish ground truth odometry", model.model_name)
    else:
        try:
            result = get_link_srv(link)
            result = result.link_state
            odom.pose.pose = result.pose
            odom.twist.twist = result.twist
            header.stamp = rospy.Time.now()
            odom.header = header
            odom_pub.publish(odom)

            t = geometry_msgs.msg.TransformStamped()

            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "ground_truth"
            t.child_frame_id = "base_link"
            t.transform.translation.x = odom.pose.pose.position.x
            t.transform.translation.y = odom.pose.pose.position.y
            t.transform.translation.z = odom.pose.pose.position.z
            t.transform.rotation.x = odom.pose.pose.orientation.x
            t.transform.rotation.y = odom.pose.pose.orientation.y
            t.transform.rotation.z = odom.pose.pose.orientation.z
            t.transform.rotation.w = odom.pose.pose.orientation.w

            br.sendTransform(t)
        
        except:
            print("error happends in publish_ground_truth_odom.py")
    r.sleep()
