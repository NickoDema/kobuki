#!/usr/bin/env python

##   agv_control.py
##   Created on: 11.09.2018
##           By: Nikoaly Dema
##        Email: ndema2301@gmail.com
##               goTRG | spb

import time
import json
import thread
from math import sin, cos, radians, fabs, atan2, sqrt

import rospy
import tf
import tf2_ros
import actionlib

from move_base_msgs.msg import *
from geometry_msgs.msg import Pose2D, PoseWithCovarianceStamped
from std_msgs.msg import String, Empty
from std_srvs.srv import Empty

from actionlib_msgs.msg import *

from agv_navigation import points
from agv_msgs.srv import *
from agv_msgs.msg import *

class AgvControl():

    def __init__(self):

        rospy.init_node('agv_control')

        self.tf_listener = tf.TransformListener()

        rospy.Subscriber('battery', BatteryStateRaw, self.battery_cb)

        self.agv_state_pub = rospy.Publisher('agv_state', AgvState, queue_size=1)

        self.robot_location_rate = 0.2
        rospy.Timer(rospy.Duration(self.robot_location_rate), self.robot_location_cb)

        self.follow_on_srv =    rospy.Service('follow_on',    Trigger,    self.follow_on_cb)
        self.stop_srv =         rospy.Service('agv_stop',     Trigger,    self.stop_cb)

        self.move_to_pose_as = actionlib.SimpleActionServer('move_to_pose_as',
                                                            MoveToPoseAction,
                                                            execute_cb=self.move_to_pose_as_cb,
                                                            auto_start = False)

        self.follow_as = actionlib.SimpleActionServer('follow_as',
                                                      FollowAction,
                                                      execute_cb=self.follow_as_cb,
                                                      auto_start = False)

        self.agv_state_lock = thread.allocate_lock()
        self.agv_state = AgvState()
        self.agv_state.status = AgvState.READY

        self.agv_state.battery.status = BatteryState.DISCHARGING
        self.agv_state.battery.charge_level = 100

        self.move_to_pose_as.start()
        self.follow_as.start()


    def send_goal_to_move_base(self, move_base_client, x, y, q):

        if move_base_client.wait_for_server(rospy.Duration(4)):
            move_base_goal = MoveBaseGoal()
            move_base_goal.target_pose.pose.position.x = x
            move_base_goal.target_pose.pose.position.y = y
            move_base_goal.target_pose.pose.orientation.z = q[2]
            move_base_goal.target_pose.pose.orientation.w = q[3]
            move_base_goal.target_pose.header.frame_id = 'map'
            move_base_goal.target_pose.header.stamp = rospy.Time.now()

            move_base_client.send_goal(move_base_goal)
            return True
        else:
            return False


    def move_to_pose_as_cb(self, goal):

        feedback = MoveToPoseFeedback()
        result = MoveToPoseResult()

        move_base_ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        q = tf.transformations.quaternion_from_euler(0, 0, goal.pose.theta)

        if self.send_goal_to_move_base(move_base_ac, goal.pose.x, goal.pose.y, q):

            with self.agv_state_lock:
                self.agv_state.status = AgvState.BUSY
                self.agv_state.command = AgvState.MOVE

            feedback_rate = rospy.Rate(2)
            while not rospy.is_shutdown():

                if self.move_to_pose_as.is_preempt_requested():
                    move_base_ac.cancel_goal()
                    result.message = 'Goal is preempted (stopped)'
                    result.move_base_goal_status = MoveToPoseResult.PREEMPTED
                    self.move_to_pose_as.set_preempted(result)
                    break

                move_base_goal_state = move_base_ac.get_state()
                move_base_goal_is_active = ((move_base_goal_state == GoalStatus.ACTIVE)     or
                                            (move_base_goal_state == GoalStatus.PENDING)    or
                                            (move_base_goal_state == GoalStatus.PREEMPTING) or
                                            (move_base_goal_state == GoalStatus.RECALLING))

                if move_base_goal_is_active:
                    #TODO PAUSE
                    feedback.progress = 50
                    self.move_to_pose_as.publish_feedback(feedback)
                    feedback_rate.sleep()

                else:
                    if (move_base_goal_state == GoalStatus.SUCCEEDED):
                        result.move_base_goal_status = MoveToPoseResult.SUCCEEDED
                        feedback.progress = 100
                        self.move_to_pose_as.publish_feedback(feedback)
                        self.move_to_pose_as.set_succeeded(result)
                        break

                    elif (move_base_goal_state == GoalStatus.PREEMPTED):
                        result.move_base_goal_status = MoveToPoseResult.PREEMPTED
                        result.message = 'Move_base goal is preempted (stopped) by another client'
                        self.move_to_pose_as.set_aborted(result)
                        break

                    else:
                        result.move_base_goal_status = MoveToPoseResult.ABORTED
                        result.message = 'Move_base goal is aborted for an unknown reason'
                        self.move_to_pose_as.set_aborted(result)
                        break

            with self.agv_state_lock:
                self.agv_state.status = AgvState.READY
                self.agv_state.command = AgvState.WAIT_FOR_COMMAND

        else:
            rospy.logerr("[AGV control]: Timeout for move_base action server")
            result.move_base_goal_status = MoveToPoseResult.REJECTED
            result.message = "Goal aborted because of timeout for move_base action server"
            self.move_to_pose_as.set_aborted(result)


    def follow_as_cb(self, goal):

        feedback = FollowFeedback()
        result = FollowResult()

        try:
            navi_follow_on_srv = rospy.ServiceProxy('navi_follow_on', Trigger)
            navi_follow_res = navi_follow_on_srv()
            navi_is_active = navi_follow_res.success
        except Exception as e:
            navi_is_active = False
            print(e)

        try:
            detector_follow_on_srv = rospy.ServiceProxy('start_follow_camera_only_system', Trigger)
            detector_follow_res = detector_follow_on_srv()
            detector_is_active = detector_follow_res.success
        except Exception as e:
            detector_is_active = False
            print(e)

        if (navi_is_active and detector_is_active):
            with self.agv_state_lock:
                self.agv_state.status = AgvState.BUSY
                self.agv_state.command = AgvState.FOLLOW

            feedback_rate = rospy.Rate(2)
            while not rospy.is_shutdown():

                if self.follow_as.is_preempt_requested():
                    self.stop_cb()
                    result.message = 'Follow is preempted (stopped)'
                    result.goal_status = FollowResult.STOPPED
                    self.follow_as.set_preempted(result)
                    break

                # get an info about the follow nodes through the corresponding topics
                feedback.status = FollowFeedback.ACTIVE
                self.follow_as.publish_feedback(feedback)
                feedback_rate.sleep()

            with self.agv_state_lock:
                self.agv_state.status = AgvState.READY
                self.agv_state.command = AgvState.WAIT_FOR_COMMAND

        elif (not navi_is_active):
            self.stop_cb()
            rospy.logerr("[AGV control]: Navi follow node doesnt respond.")
            result.goal_status = FollowResult.REJECTED
            result.message = "Navi follow node doesnt respond"
            self.follow_as.set_aborted(result)

        else:
            self.stop_cb()
            rospy.logerr("[AGV control]: Follow detection node doesnt respond.")
            result.goal_status = FollowResult.REJECTED
            result.message = "Follow detection node doesnt respon"
            self.follow_as.set_aborted(result)


    def follow_on_cb(self, req):

        navi_follow_on_srv = rospy.ServiceProxy('navi_follow_on', Trigger)
        navi_follow_res = navi_follow_on_srv()

        detector_follow_on_srv = rospy.ServiceProxy('start_follow_camera_only_system', Trigger)
        detector_follow_res = detector_follow_on_srv()

        if navi_follow_res.success:
            with self.agv_state_lock:
                self.agv_state.status = AgvState.BUSY
                self.agv_state.command = AgvState.FOLLOW

        res = TriggerResponse()
        res.success = navi_follow_res.success

        return res


    def stop_cb(self, req = None):

        res = TriggerResponse()

        if self.agv_state.command == AgvState.MOVE:
            self.stop_move_base()

        if self.agv_state.command == AgvState.FOLLOW:

            navi_follow_off_srv = rospy.ServiceProxy('navi_follow_off', Trigger)
            navi_follow_res = navi_follow_off_srv()

            detector_follow_off_srv = rospy.ServiceProxy('pause_follow_system', Trigger)
            detector_follow_res = detector_follow_off_srv()

            res.success = True

        with self.agv_state_lock:
            self.agv_state.status = AgvState.READY
            self.agv_state.command = AgvState.WAIT_FOR_COMMAND

        return res


    def stop_move_base(self):
        move_base_ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        try:
            move_base_ac.wait_for_server(5)
            move_base_ac.cancel_all_goals()
            return True
        except:
            rospy.logwarn("[AGV control]: Can't cancel a goal for move_base")
            return False


    def robot_location_cb(self, event):
        try:
            pos, q = self.tf_listener.lookupTransform("map", "base_link", rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("[AGV control]: Can\'t get map - base_link transform")
            rospy.sleep(2.)
            return

        euler = tf.transformations.euler_from_quaternion(q)
        with self.agv_state_lock:
            self.agv_state.pose.x = pos[0]
            self.agv_state.pose.y = pos[1]
            self.agv_state.pose.theta = euler[2]


    def battery_cb(self, msg):

        battery_error_msg = self.check_battery_for_errors(msg)

        with self.agv_state_lock:
            if msg.charger_connected:
                if msg.battery_charged:
                    self.agv_state.battery.status = BatteryState.CHARGED
                else:
                    self.agv_state.battery.status = BatteryState.CHARGING
            else:
                self.agv_state.battery.status = BatteryState.DISCHARGING

            self.agv_state.battery.charge_level = msg.charge_level
            self.agv_state.battery.errors = battery_error_msg
            if msg.has_errors:
                self.agv_state.has_critical_error = True
            else:
                self.agv_state.has_critical_error = False


    def check_battery_for_errors(self, msg):
        battery_error_msg = AgvError()
        if msg.has_errors:
            battery_error_msg.status = AgvError.CRITICAL
            error_message = 'Problem with BMS: '
            error_raw = {'source': 'BMS',
                         'errors': []}

            for error in msg.errors:
                if error == BatteryStateRaw.OVERVOLTAGE:
                    error_message += 'overvoltage (' + msg.voltage + 'V) '
                    error_raw['errors'].append({'code': BatteryStateRaw.OVERVOLTAGE,
                                                'type': 'overvoltage',
                                                'value': msg.voltage})
                if error == BatteryStateRaw.UNDERVOLTAGE:
                    error_message += 'undervoltage (' + msg.voltage + 'V) '
                    error_raw['errors'].append({'code': BatteryStateRaw.UNDERVOLTAGE,
                                                'type': 'undervoltage',
                                                'value': msg.voltage})
                if error == BatteryStateRaw.OVERHEAT:
                    error_message += 'battery overheat (max temp. is ' + msg.max_temperature + 'C) '
                    error_raw['errors'].append({'code': BatteryStateRaw.OVERHEAT,
                                                'type': 'battery_overheat',
                                                'value': msg.max_temperature})
                if error == BatteryStateRaw.OVERCURRENT:
                    error_message += 'overcurrent (' + msg.current + 'A) '
                    error_raw['errors'].append({'code': BatteryStateRaw.OVERCURRENT,
                                                'type': 'overcurrent',
                                                'value': msg.current})
                if error == BatteryStateRaw.LOW_TEMPERATURE:
                    error_message += 'low battery temperature (max temp. is ' + msg.max_temperature + 'C) '
                    error_raw['errors'].append({'code': BatteryStateRaw.LOW_TEMPERATURE,
                                                'type': 'low_battery_temperature',
                                                'value': msg.max_temperature})
                if error == BatteryStateRaw.LOST_LOGIC_CONNECTION:
                    error_message += 'Logic module communication error '
                    error_raw['errors'].append({'code': BatteryStateRaw.LOST_LOGIC_CONNECTION,
                                                'type': 'inter_communication',
                                                'value': None})
                if error == BatteryStateRaw.COUNT_CELLS_ERROR:
                    error_message += 'cells control (count error) '
                    error_raw['errors'].append({'code': BatteryStateRaw.COUNT_CELLS_ERROR,
                                                'type': 'cell_count',
                                                'value': None})

            battery_error_msg.message = error_message
            battery_error_msg.raw = json.dumps(error_raw)
        return battery_error_msg


    def spin(self):
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            with self.agv_state_lock:
                self.agv_state_pub.publish(self.agv_state)
            rate.sleep()

if __name__ == '__main__':
    agv_control = AgvControl()
    agv_control.spin()
