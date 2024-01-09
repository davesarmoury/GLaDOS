#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import TransformStamped, PoseStamped
from tf.transformations import *
import tf
import numpy as np
from zed_interfaces.msg import ObjectsStamped
import math
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import copy

HEAD_DISTANCE = 0.6

limits = [ [-1.57, 1.57], [1.76, 2.50], [-1.76, -0.50], [-0.90, -0.90], [-0.10, 0.10], [-3.14, 3.14] ]
mount = [0, 1.759292, -1.700000, -0.9, 0, 0]
joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

def arm_command(pos, time):
    msg = JointTrajectory()
    msg.header.stamp = rospy.Time.now()

    msg.joint_names = joint_names

    point_msg = JointTrajectoryPoint()
    point_msg.time_from_start = rospy.Time.from_sec(time)

    point_msg.positions = pos
    msg.points = []
    msg.points.append(point_msg)

    arm_pub.publish(msg)

def j_limits(values):
    j_values = []
    for i in range(len(values)):
        min_val = limits[i][0]
        max_val = limits[i][1]
        j_values.append(max(min(max_val, values[i]), min_val))

    return j_values

def det_callback(msg):
    global broadcaster, world_to_cam, view_to_world, world_to_view

    if len(msg.objects) > 0:
        min_distance = 99999.0
        closest_obj = None

        for o in msg.objects:
            length = np.linalg.norm(np.array([o.position[0], o.position[1], o.position[2]])) # Technically closest to the camera, but meh
            if length < min_distance:
                closest_obj = o
                min_distance = length

        o_mtx = compose_matrix(translate=[o.position[0], o.position[1], o.bounding_box_3d.corners[0].kp[2]])
        world_to_o = np.matmul(world_to_cam, o_mtx)
        view_to_o = np.matmul(view_to_world, world_to_o)
        scale, shear, angles, translate, perspective = decompose_matrix(view_to_o)

        horizontal_distance = math.sqrt(translate[0]*translate[0] + translate[1]*translate[1])
        yaw = math.atan2(translate[1], translate[0])
        pitch = math.atan2(translate[2],horizontal_distance) * -1.0
        h_mtx = compose_matrix(translate=[HEAD_DISTANCE, 0, 0])
        t_mtx = compose_matrix(angles=[0,pitch,yaw])

        j_angles = copy.deepcopy(mount)
        j_angles[0] = yaw + 0.25 * yaw
        j_angles[2] = j_angles[2] + pitch + 0.85
        j_angles[4] = -0.25 * yaw
        j_angles[5] = yaw/2.0
        j_angles = j_limits(j_angles)

        arm_command(j_angles, 0.5)

        t_mtx = np.matmul(t_mtx, h_mtx)
        t_mtx = np.matmul(world_to_view, t_mtx)

        scale, shear, angles2, translate2, perspective = decompose_matrix(t_mtx)
        quat = quaternion_from_euler(angles2[0], angles2[1], angles2[2])

        broadcaster.sendTransform(translate2,
                        quat,
                        rospy.Time.now(),
                        "target_" + str(o.label_id),
                        "world")

def main():
    global broadcaster, world_to_cam, view_to_world, world_to_view, arm_pub
    rospy.init_node('glados_watcher')
    broadcaster = tf.TransformBroadcaster()

    listener = tf.TransformListener()

    listener.waitForTransform('world', 'zed2i_left_camera_frame', rospy.Time(0), rospy.Duration(0.5))
    trans, rot = listener.lookupTransform('world', 'zed2i_left_camera_frame', rospy.Time(0))
    world_to_cam = compose_matrix(translate=trans, angles=euler_from_quaternion(rot))

    listener.waitForTransform('world', 'view_link', rospy.Time(0), rospy.Duration(0.5))
    trans, rot = listener.lookupTransform('world', 'view_link', rospy.Time(0))
    world_to_view = compose_matrix(translate=trans, angles=euler_from_quaternion(rot))
    view_to_world = np.linalg.inv(world_to_view)

    arm_pub = rospy.Publisher('/z1_joint_traj_controller/command', JointTrajectory, queue_size=10)
    rospy.Subscriber("/zed_node/obj_det/objects", ObjectsStamped, det_callback)

    arm_command(mount, 2.0)
    rospy.sleep(2.0)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
