#!/usr/bin/env python3
import rospy

from mujoco_ros_msgs.srv import SetMocapState, SetMocapStateRequest
from geometry_msgs.msg import PoseStamped





if __name__ == "__main__":
    rospy.init_node("mocap", anonymous=True)
    rospy.wait_for_service("/mujoco_server/set_mocap_state", 2)

    mocap_client = rospy.ServiceProxy('/mujoco_server/set_mocap_state', SetMocapState)
    mocap_msg = SetMocapStateRequest()

    p = PoseStamped()

    mocap_msg.mocap_state.name = ["mocap"]
    mocap_msg.mocap_state.pose = [p]

    # stort lower at a more adequate pose
    mocap_msg.mocap_state.pose[0].pose.position.x = 0.
    mocap_msg.mocap_state.pose[0].pose.position.y = 0.
    mocap_msg.mocap_state.pose[0].pose.position.z = 0.07
    mocap_client(mocap_msg)
   
    
    up = 1



    while not rospy.is_shutdown():
        mocap_msg.mocap_state.pose[0].pose.position.z += up * 0.0001
        if mocap_msg.mocap_state.pose[0].pose.position.z >= 0.12:
            up = -1
        elif mocap_msg.mocap_state.pose[0].pose.position.z <= 0.05:
            up = 1
        mocap_client(mocap_msg)
        rospy.sleep(0.005)
