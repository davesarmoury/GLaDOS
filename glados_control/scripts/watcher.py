#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import TransformStamped, PoseStamped
from tf.transformations import *
import tf
import numpy as np
from zed_interfaces.msg import ObjectsStamped
import math

HEAD_DISTANCE = 0.6

limits = [ [-1.57, 1.57], [1.76, 2.50], [-1.76, -0.50], [0.90, 0.90], [-0.10, 0.10], [-3.14, 3.14] ]

def det_callback(msg):
    global broadcaster, world_to_cam, view_to_world, world_to_view, last_pos

    if len(msg.objects) > 0:
        min_distance = 99999.0
        closest_obj = None

        for o in msg.objects:
            len = np.linalg.norm(np.array([o.position[0], o.position[1], o.position[2]])) # Technically closest to the camera, but meh
            if len < min_distance:
                closest_obj = o
                min_distance = len

        o_mtx = compose_matrix(translate=[o.position[0], o.position[1], o.position[2]])
        world_to_o = np.matmul(world_to_cam, o_mtx)
        view_to_o = np.matmul(view_to_world, world_to_o)
        scale, shear, angles, translate, perspective = decompose_matrix(view_to_o)

        horizontal_distance = math.sqrt(translate[0]*translate[0] + translate[1]*translate[1])
        yaw = math.atan2(translate[1], translate[0])
        pitch = math.atan2(translate[2],horizontal_distance) * -1.0
        h_mtx = compose_matrix(translate=[HEAD_DISTANCE, 0, 0])
        t_mtx = compose_matrix(angles=[0,pitch,yaw])

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
    global broadcaster, world_to_cam, view_to_world, world_to_view, last_pos
    rospy.init_node('glados_watcher')
    broadcaster = tf.TransformBroadcaster()

    listener = tf.TransformListener()

    listener.waitForTransform('world', 'zed2_left_camera_frame', rospy.Time(0), rospy.Duration(0.5))
    trans, rot = listener.lookupTransform('world', 'zed2_left_camera_frame', rospy.Time(0))
    world_to_cam = compose_matrix(translate=trans, angles=euler_from_quaternion(rot))

    listener.waitForTransform('world', 'view_link', rospy.Time(0), rospy.Duration(0.5))
    trans, rot = listener.lookupTransform('world', 'view_link', rospy.Time(0))
    world_to_view = compose_matrix(translate=trans, angles=euler_from_quaternion(rot))
    view_to_world = np.linalg.inv(world_to_view)

#    listener.waitForTransform('world', 'eye_link', rospy.Time(0), rospy.Duration(0.5))
#    trans, rot = listener.lookupTransform('world', 'eye_link', rospy.Time(0))
#    last_pos = compose_matrix(translate=trans, angles=euler_from_quaternion(rot))

    rospy.loginfo(last_pos)

    rospy.Subscriber("/zed_node/obj_det/objects", ObjectsStamped, det_callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass