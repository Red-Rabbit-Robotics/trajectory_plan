#!/usr/bin/env python

import os
import rospy
import yaml
from std_msgs.msg import Empty
from geometry_msgs.msg import Pose
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, Marker
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QFileDialog
from rqt_gui_py.plugin import Plugin
import rospkg
from std_srvs.srv import Empty as EmptySrv

class TrajectoryPlannerRqt(Plugin):

    def __init__(self, context):
        super(TrajectoryPlannerRqt, self).__init__(context)
        self.setObjectName('TrajectoryPlannerRqt')

        self._widget = QWidget()
        ui_file = os.path.join(rospkg.RosPack().get_path('trajectory_planner_rqt'), 'resource', 'trajectory_planner_rqt.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('TrajectoryPlannerRqtUi')
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

        self.plan_left_pub = rospy.Publisher('/plan_left_trajectory', Pose, queue_size=1)
        self.plan_right_pub = rospy.Publisher('/plan_right_trajectory', Pose, queue_size=1)
        self.visualize_left_pub = rospy.Publisher('/visualize_left_trajectory', Empty, queue_size=1)
        self.visualize_right_pub = rospy.Publisher('/visualize_right_trajectory', Empty, queue_size=1)
        self.publish_left_pub = rospy.Publisher('/publish_left_trajectory', Empty, queue_size=1)
        self.publish_right_pub = rospy.Publisher('/publish_right_trajectory', Empty, queue_size=1)

        self._widget.planLeftButton.clicked.connect(self._on_plan_left_button_clicked)
        self._widget.planRightButton.clicked.connect(self._on_plan_right_button_clicked)
        self._widget.visualizeLeftButton.clicked.connect(self._on_visualize_left_button_clicked)
        self._widget.visualizeRightButton.clicked.connect(self._on_visualize_right_button_clicked)
        self._widget.publishLeftButton.clicked.connect(self._on_publish_left_button_clicked)
        self._widget.publishRightButton.clicked.connect(self._on_publish_right_button_clicked)
        self._widget.loadButton.clicked.connect(self._on_load_button_clicked)
        self._widget.saveButton.clicked.connect(self._on_save_button_clicked)

        self._config_file = os.path.join(rospkg.RosPack().get_path('trajectory_planner'), 'config', 'trajectory_planner.yaml')
        self.load_config()

        self.server = InteractiveMarkerServer("interactive_marker_server")
        self.createMarkers()
        rospy.Subscriber("/interactive_marker_feedback", InteractiveMarkerFeedback, self.processFeedback)

        self.reload_config_srv = rospy.ServiceProxy('/reload_config', EmptySrv)

    def _on_plan_left_button_clicked(self):
        self.plan_left_pub.publish(self.left_target_pose)

    def _on_plan_right_button_clicked(self):
        self.plan_right_pub.publish(self.right_target_pose)

    def _on_visualize_left_button_clicked(self):
        self.visualize_left_pub.publish(Empty())

    def _on_visualize_right_button_clicked(self):
        self.visualize_right_pub.publish(Empty())

    def _on_publish_left_button_clicked(self):
        self.publish_left_pub.publish(Empty())

    def _on_publish_right_button_clicked(self):
        self.publish_right_pub.publish(Empty())

    def _on_load_button_clicked(self):
        options = QFileDialog.Options()
        file_name, _ = QFileDialog.getOpenFileName(self._widget, "Load Config File", "", "YAML Files (*.yaml);;All Files (*)", options=options)
        if file_name:
            self._config_file = file_name
            self.load_config()

    def _on_save_button_clicked(self):
        options = QFileDialog.Options()
        file_name, _ = QFileDialog.getSaveFileName(self._widget, "Save Config File", "", "YAML Files (*.yaml);;All Files (*)", options=options)
        if file_name:
            self._config_file = file_name
            self.save_config()
            self.reload_config()

    def load_config(self):
        with open(self._config_file, 'r') as file:
            config = yaml.safe_load(file)
            self._widget.urdfFileLineEdit.setText(config.get('urdf_file', ''))
            self._widget.chainStartLeftLineEdit.setText(config['left_arm'].get('chain_start', ''))
            self._widget.chainEndLeftLineEdit.setText(config['left_arm'].get('chain_end', ''))
            self._widget.chainStartRightLineEdit.setText(config['right_arm'].get('chain_start', ''))
            self._widget.chainEndRightLineEdit.setText(config['right_arm'].get('chain_end', ''))
            self._widget.collisionRadiusSpinBox.setValue(config.get('collision_radius', 0.1))
            self._widget.timeSpinBox.setValue(config.get('time', 10.0))
            self._widget.numWaypointsSpinBox.setValue(config.get('num_waypoints', 10))
            self.left_joint_names = config['left_arm'].get('joint_names', [])
            self.right_joint_names = config['right_arm'].get('joint_names', [])

    def save_config(self):
        config = {
            'urdf_file': self._widget.urdfFileLineEdit.text(),
            'left_arm': {
                'chain_start': self._widget.chainStartLeftLineEdit.text(),
                'chain_end': self._widget.chainEndLeftLineEdit.text(),
                'joint_names': self.left_joint_names
            },
            'right_arm': {
                'chain_start': self._widget.chainStartRightLineEdit.text(),
                'chain_end': self._widget.chainEndRightLineEdit.text(),
                'joint_names': self.right_joint_names
            },
            'collision_radius': self._widget.collisionRadiusSpinBox.value(),
            'time': self._widget.timeSpinBox.value(),
            'num_waypoints': self._widget.numWaypointsSpinBox.value()
        }
        with open(self._config_file, 'w') as file:
            yaml.safe_dump(config, file)

    def reload_config(self):
        try:
            self.reload_config_srv()
            rospy.loginfo("Configuration reloaded successfully.")
        except rospy.ServiceException as e:
            rospy.logerr("Failed to reload configuration: %s", e)

    def createMarkers(self):
        self.createMarker("left_target_marker", 0.3, 0.3, "Left Arm Target Pose")
        self.createMarker("right_target_marker", 0.3, -0.3, "Right Arm Target Pose")

    def createMarker(self, name, x, y, description):
        marker = InteractiveMarker()
        marker.header.frame_id =self._widget.chainStartLeftLineEdit.text()
        marker.name = name
        marker.description = description
        marker.scale = 0.15

        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = -0.3
        pose.orientation.x = 0.0
        pose.orientation.y = -0.5227
        pose.orientation.z = 0.0
        pose.orientation.w = 0.8525 # r=0, p=-1.1, y=0
        marker.pose = pose

        # Create a 6-DOF control
        control_xr = InteractiveMarkerControl()
        control_xr.orientation.w = 1
        control_xr.orientation.x = 1
        control_xr.orientation.y = 0
        control_xr.orientation.z = 0
        control_xr.name = "rotate_x"
        control_xr.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        marker.controls.append(control_xr)

        control_xm = InteractiveMarkerControl()
        control_xm.orientation.w = 1
        control_xm.orientation.x = 1
        control_xm.orientation.y = 0
        control_xm.orientation.z = 0
        control_xm.name = "move_x"
        control_xm.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        marker.controls.append(control_xm)

        control_yr = InteractiveMarkerControl()
        control_yr.orientation.w = 1
        control_yr.orientation.x = 0
        control_yr.orientation.y = 1
        control_yr.orientation.z = 0
        control_yr.name = "rotate_y"
        control_yr.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        marker.controls.append(control_yr)

        control_ym = InteractiveMarkerControl()
        control_ym.orientation.w = 1
        control_ym.orientation.x = 0
        control_ym.orientation.y = 1
        control_ym.orientation.z = 0
        control_ym.name = "move_y"
        control_ym.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        marker.controls.append(control_ym)

        control_zr = InteractiveMarkerControl()
        control_zr.orientation.w = 1
        control_zr.orientation.x = 0
        control_zr.orientation.y = 0
        control_zr.orientation.z = 1
        control_zr.name = "rotate_z"
        control_zr.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        marker.controls.append(control_zr)

        control_zm = InteractiveMarkerControl()
        control_zm.orientation.w = 1
        control_zm.orientation.x = 0
        control_zm.orientation.y = 0
        control_zm.orientation.z = 1
        control_zm.name = "move_z"
        control_zm.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        marker.controls.append(control_zm)

        self.server.insert(marker, self.processFeedback)
        self.server.applyChanges()

    def processFeedback(self, feedback):
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            if feedback.marker_name == "left_target_marker":
                self.left_target_pose = feedback.pose
            elif feedback.marker_name == "right_target_marker":
                self.right_target_pose = feedback.pose

if __name__ == '__main__':
    import sys
    rospy.init_node('trajectory_planner_rqt')
    app = TrajectoryPlannerRqt()
    rospy.spin()

