#!/usr/bin/env python
import sys
# sys.path.insert(0, '/home/hdh5/anaconda3/lib/python3.8/site-packages')

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8MultiArray

# from PyQt5.QtWidgets import QApplication, QWidget
# from PyQt5.Qt import QKeyEvent, Qt

from python_qt_binding.QtWidgets import QApplication, QWidget
# from python_qt_binding.Qt import QKeyEvent
from python_qt_binding.QtCore import Qt


class KeyboardWidget(QWidget):
    def __init__(self, parent=None):
        super(KeyboardWidget, self).__init__(parent)

        self.delta_x = 0
        self.delta_y = 0
        self.delta_z = 0
        self.delta_pitch = 0
        self.delta_yaw = 0
        self.delta_roll = 0

        rospy.init_node("teleop_twist_keyboard")
        self.cmd_pub = rospy.Publisher("/cmd_tf", Int8MultiArray, queue_size=3)
        self.pub_timer = rospy.Timer(rospy.Duration(nsecs=100000000), self.pub_key)
        pass

    def pub_key(self, time_event):
        # print("forward speed: ", self.forward_speed)
        # print("turn speed: ", self.turn_speed)
        data = []
        data.append(self.delta_x)
        data.append(self.delta_y)
        data.append(self.delta_z)
        data.append(self.delta_pitch)
        data.append(self.delta_yaw)
        data.append(self.delta_roll)
        self.cmd_pub.publish(Int8MultiArray(data=data))
        pass

    def keyPressEvent(self, event):
        if not event.isAutoRepeat():
            key = event.key()
            if key == Qt.Key_Q or key == Qt.Key_A:
                self.delta_x = 1 if key == Qt.Key_Q else -1
            if key == Qt.Key_W or key == Qt.Key_S:
                self.delta_y = 1 if key == Qt.Key_W else -1
            if key == Qt.Key_E or key == Qt.Key_D:
                self.delta_z = 1 if key == Qt.Key_E else -1
            if key == Qt.Key_R or key == Qt.Key_F:
                self.delta_pitch = 1 if key == Qt.Key_R else -1
            if key == Qt.Key_T or key == Qt.Key_G:
                self.delta_yaw = 1 if key == Qt.Key_T else -1
            if key == Qt.Key_Y or key == Qt.Key_H:
                self.delta_roll = 1 if key == Qt.Key_Y else -1
        return QWidget.keyPressEvent(self, event)

    def keyReleaseEvent(self, event):
        if not event.isAutoRepeat():
            key = event.key()
            if key == Qt.Key_Q or key == Qt.Key_A:
                self.delta_x = 0
            if key == Qt.Key_W or key == Qt.Key_S:
                self.delta_y = 0
            if key == Qt.Key_E or key == Qt.Key_D:
                self.delta_z = 0
            if key == Qt.Key_R or key == Qt.Key_F:
                self.delta_pitch = 0
            if key == Qt.Key_T or key == Qt.Key_G:
                self.delta_yaw = 0
            if key == Qt.Key_Y or key == Qt.Key_H:
                self.delta_roll = 0

        return QWidget.keyReleaseEvent(self, event)


if __name__ == '__main__':
    app = QApplication(sys.argv)

    widget = KeyboardWidget()
    widget.setFixedWidth(240)
    widget.setFixedHeight(150)
    widget.show()

    app.exec()
    pass
