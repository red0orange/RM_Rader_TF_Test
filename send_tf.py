# -*- coding: utf-8 -*-
# @Time    : 2021/7/24 上午10:54
# @Author  : red0orange
# @File    : send_tf.py
# @Content :
import os
import rospy
import tf2_ros
import numpy as np
import geometry_msgs.msg
from std_msgs.msg import Int8MultiArray

from transformations import quaternion_from_euler, quaternion_from_matrix, euler_matrix
from utils import get_homo_matrix, get_RT_from_homo_matrix, get_zero_translate_homo


class TFSender(object):
    def __init__(self):
        # 初始化订阅键盘控制话题、TF发布者
        self.topic_name = "/cmd_tf"
        self.sub = rospy.Subscriber(self.topic_name, Int8MultiArray, self.keyboard_callback)
        self.tf_br = tf2_ros.TransformBroadcaster()

        # 初始化键盘接收delta增益系数
        self.translate_gain = 0.1
        self.rotate_gain = 0.05
        # 初始化位姿6自由度描述变量
        self.cur_x = 0
        self.cur_y = 0
        self.cur_z = 0
        self.cur_rotate_by_x = 0
        self.cur_rotate_by_y = 0
        self.cur_rotate_by_z = 0

        # 初始化TF定时发布定时器
        self.pub_timer = rospy.Timer(rospy.Duration(nsecs=100000000), self.pub_tf)
        pass

    def keyboard_callback(self, msg):
        data = msg.data
        delta_x, delta_y, delta_z, delta_pitch, delta_yaw, delta_roll = data
        self.cur_x += delta_x * self.translate_gain
        self.cur_y += delta_y * self.translate_gain
        self.cur_z += delta_z * self.translate_gain
        self.cur_rotate_by_x += delta_pitch * self.rotate_gain
        self.cur_rotate_by_y += delta_yaw * self.rotate_gain
        self.cur_rotate_by_z += delta_roll * self.rotate_gain
        pass

    def pub_tf(self, time_event):
        # # 由于控制TF先平移后旋转比较方便，因此得到先平移后旋转分别的RT
        # # 欧拉角转R，旋转顺序是定轴x->y->z
        # rotated_matrix = euler_matrix(self.cur_rotate_by_x, self.cur_rotate_by_y, self.cur_rotate_by_z, axes='sxyz')[:3, :3]
        # # T
        # translate_vector = np.array([self.cur_x, self.cur_y, self.cur_z])
        #
        # # 分别得到各自的homo_matrix
        # # 平移+零旋转
        # zero_rotated_matrix = euler_matrix(0, 0, 0)[:3, :3]
        # homo_matrix_T = get_homo_matrix(zero_rotated_matrix, translate_vector)
        # # 零平移+旋转
        # zero_translate_vector = np.array([0, 0, 0])
        # homo_matrix_R = get_homo_matrix(rotated_matrix, zero_translate_vector)
        # # 得到合并的homo_matrix，先平移后旋转，右往左写
        # combine_homo_matrix = homo_matrix_R.dot(homo_matrix_T)
        # # 得到对应的先旋转后平移的R、T
        # R, T = get_RT_from_homo_matrix(combine_homo_matrix)
        # t = geometry_msgs.msg.TransformStamped()
        # t.header.stamp = rospy.Time.now()
        # t.header.frame_id = "world"
        # t.child_frame_id = "base_link"
        # t.transform.translation.x = T[0]
        # t.transform.translation.y = T[1]
        # t.transform.translation.z = T[2]
        # q = quaternion_from_matrix(get_zero_translate_homo(R))
        # t.transform.rotation.x = q[0]
        # t.transform.rotation.y = q[1]
        # t.transform.rotation.z = q[2]
        # t.transform.rotation.w = q[3]
        # self.tf_br.sendTransform(t)

        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "world"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.cur_x
        t.transform.translation.y = self.cur_y
        t.transform.translation.z = self.cur_z
        q = quaternion_from_matrix(euler_matrix(self.cur_rotate_by_x, self.cur_rotate_by_y, self.cur_rotate_by_z, "sxyz"))
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_br.sendTransform(t)
        pass


if __name__ == '__main__':
    rospy.init_node('tf_sender')
    tf_sender = TFSender()
    rospy.spin()
    pass