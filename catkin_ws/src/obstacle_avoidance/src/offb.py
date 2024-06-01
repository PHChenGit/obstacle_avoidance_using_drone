#! /usr/bin/python3

import time

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np


current_state = State()
pose = PoseStamped()

def state_cb(msg):
    global current_state
    current_state = msg

def update_position(msg):
    new_pose = msg
    rospy.loginfo(f'new pose: {new_pose}')

    # pose.pose.position.x += msg.linear.x
    # pose.pose.position.y += msg.linear.y
    # pose.pose.position.z += msg.linear.z

    # Retrieve current orientation from the pose
    current_orientation = [pose.pose.orientation.x,
                           pose.pose.orientation.y,
                           pose.pose.orientation.z,
                           pose.pose.orientation.w]
    
    # Convert quaternion to Euler angles
    current_euler = euler_from_quaternion(current_orientation)
    current_roll = current_euler[0]
    current_pitch = current_euler[1]
    current_yaw = current_euler[2]

    vx = msg.linear.x * np.cos(current_yaw) - msg.linear.y * np.sin(current_yaw)
    vy = msg.linear.x * np.sin(current_yaw) + msg.linear.y * np.cos(current_yaw)

    pose.pose.position.x += vx
    pose.pose.position.y += vy
    pose.pose.position.z += msg.linear.z

    new_yaw = current_yaw + msg.angular.z
    new_orientation = quaternion_from_euler(current_roll, current_pitch, new_yaw)

    pose.pose.orientation.x = new_orientation[0]
    pose.pose.orientation.y = new_orientation[1]
    pose.pose.orientation.z = new_orientation[2]
    pose.pose.orientation.w = new_orientation[3]

    # if (new_pose.linear.x == 0 and new_pose.linear.y == 0
    #     and new_pose.linear.z == 0
    #     and new_pose.angular.x == 0
    #     and new_pose.angular.y == 0
    #     and new_pose.angular.z == 0):
    #     pose.pose.position.x = pose.pose.position.x
    #     pose.pose.position.y = pose.pose.position.y
    #     pose.pose.position.z = pose.pose.position.z
    # else:
    #     pose.pose.position.x = pose.pose.position.x + new_pose.linear.x
    #     pose.pose.position.y = pose.pose.position.y + new_pose.linear.y
    #     pose.pose.position.z = pose.pose.position.z + new_pose.linear.z

    
    # local_pos_pub.publish(pose)
    # rospy.loginfo(f"pose x: {pose.pose.position.x} y: {pose.pose.position.y} z: {pose.pose.position.z}")
    


if __name__ == "__main__":
    rospy.loginfo("Running off_node")
    rospy.init_node("offb_node_py")

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)

    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)

    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    pose_sub = rospy.Subscriber('/cmd_vel', Twist, callback=update_position)


    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 2

    # Send a few setpoints before starting
    for i in range(100):
        if(rospy.is_shutdown()):
            break

        local_pos_pub.publish(pose)
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()
    rospy.loginfo("Already set mode")

    while(not rospy.is_shutdown()):
        if current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")

            last_req = rospy.Time.now()
        
        else:
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")

                last_req = rospy.Time.now()
        
        
        # if current_state.mode == 'OFFBOARD' and current_state.armed is True and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
        #     rospy.loginfo('flying')
        #     break
        #     # pose.pose.position.x
        #     # last_req = rospy.Time.now()


        # rospy.loginfo(f"pose x: {pose.pose.position.x} y: {pose.pose.position.y} z: {pose.pose.position.z}")
        local_pos_pub.publish(pose)

        rate.sleep()

