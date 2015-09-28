#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import socket
import re

import tf2_ros
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, PoseStamped, Vector3, TransformStamped, Quaternion
from tf.transformations import quaternion_from_matrix

DEFAULT_NODE_NAME = 'qp_client'
DEFAULT_POSES_TOPIC_NAME = '/qp_poses'
DEFAULT_SERVER_IP = '10.249.255.199'
DEFAULT_SERVER_PORT = 5555
DEFAULT_BUFFER_SIZE = 1024
DEFAULT_MARKER_NAMES = ['P000']
DEFAULT_CAMERA_FRAME_ID = 'camera_frame'

PARAM_NAME_SERVER_IP = '/server_ip'
PARAM_NAME_SERVER_PORT = '/server_port'
PARAM_NAME_BUFFER_SIZE = '/buffer_size'
PARAM_NAME_MARKER_NAMES = '/marker_names'
PARAM_NAME_POSES_TOPIC_NAME = '/topic_name'
PARAM_NAME_CAMERA_FRAME_ID = '/camera_frame_id'


class QPClient(object):
    mm2m = 0.001

    def __init__(self):
        self._server_ip = rospy.get_param(PARAM_NAME_SERVER_IP,
                                          default=DEFAULT_SERVER_IP)
        self._server_port = rospy.get_param(PARAM_NAME_SERVER_PORT,
                                            default=DEFAULT_SERVER_PORT)
        self._buffer_size = rospy.get_param(PARAM_NAME_BUFFER_SIZE,
                                            default=DEFAULT_BUFFER_SIZE)
        self._marker_names = rospy.get_param(PARAM_NAME_MARKER_NAMES,
                                             default=DEFAULT_MARKER_NAMES)
        self._topic_name = rospy.get_param(PARAM_NAME_POSES_TOPIC_NAME,
                                           default=DEFAULT_POSES_TOPIC_NAME)
        self._camera_frame_id = rospy.get_param(PARAM_NAME_CAMERA_FRAME_ID,
                                                default=DEFAULT_CAMERA_FRAME_ID)

        self._splitter = re.compile(r"[\n,]")
        self._tf_broadcaster = tf2_ros.TransformBroadcaster()
        self._pose_publishers, self._qp_poses, self._qp_tfs = [], [], []
        for name in self._marker_names:
            self._pose_publishers.append(rospy.Publisher(self._topic_name + '_' + name,
                                                         PoseStamped,
                                                         queue_size=5))
            self._qp_poses.append(PoseStamped(header=Header(stamp=rospy.Time.now(),
                                                            frame_id=name)))
            self._qp_tfs.append(TransformStamped(header=Header(stamp=rospy.Time.now(),
                                                               frame_id=self._camera_frame_id),
                                                 child_frame_id=name))

    def __del__(self):
        if not self._is_closed:
            self.close()

    def activate(self):
        self._clientsock = socket.socket(socket.AF_INET,
                                         socket.SOCK_STREAM)
        self._clientsock.connect((self._server_ip, self._server_port))
        self._is_closed = False
        rospy.loginfo('QPClient is activated!')
        return self

    def update(self):
        recv_str = self._clientsock.recv(self._buffer_size)
        recv_data = self._splitter.split(recv_str)

        if self._check_valid_data(recv_data):
            name, pose = self._unpack_recv_data(recv_data)
            id = self._marker_names.index(name)
            self._qp_poses[id].header.stamp = self._qp_tfs[id].header.stamp = rospy.Time.now()
            self._qp_poses[id].pose = pose
            self._qp_tfs[id].transform.translation = pose.position
            self._qp_tfs[id].transform.rotation = pose.orientation
        return self

    def publish_data(self):
        for pose, pub in zip(self._qp_poses, self._pose_publishers):
            pub.publish(pose)
        self._tf_broadcaster.sendTransform(self._qp_tfs)

    def close(self):
        if not self._is_closed:
            self._clientsock.close()
            self._is_closed = True


    def _check_valid_data(self, recv_data):
        return recv_data[0] in self._marker_names

    def _unpack_recv_data(self, recv_data):
        name = recv_data[0]
        q = quaternion_from_matrix(self._matrix_from_list(recv_data[1:]))
        return name, Pose(position=Vector3(x=float(recv_data[13])*self.mm2m,
                                           y=float(recv_data[14])*self.mm2m,
                                           z=float(recv_data[15])*self.mm2m),
                          orientation=Quaternion(x=q[0], y=q[1],
                                                 z=q[2], w=q[3]))

    def _matrix_from_list(self, lst):
        matrix = [[0 for i in range(4)] for j in range(4)]
        for i, j in [(i, j) for i in range(4) for j in range(4)]:
            matrix[j][i] = float(lst[4 * i + j])
        return matrix


# --------------------------------------------
if __name__ == '__main__':
    rospy.init_node(DEFAULT_NODE_NAME, anonymous=True)
    rate_mgr = rospy.Rate(100)  # Hz

    qpclient = QPClient().activate()

    while not rospy.is_shutdown():
        qpclient.update().publish_data()
        rate_mgr.sleep()

    qpclient.close()
