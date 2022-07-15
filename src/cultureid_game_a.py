#!/usr/bin/env python

import threading
import rospy
import numpy as np
from smach import State,StateMachine
from std_msgs.msg import Empty
from std_msgs.msg import Int8
import math
import rospkg
import time
import os
import sys
import subprocess


################################################################################
################################################################################
class GotoInitPose(State):

  ############################################################################
  def __init__(self):
    State.__init__(self, outcomes=['success'], input_keys=['g_counter'], output_keys=['g_counter'])

    # Topics. They should match those
    # of pkg `cultureid_waypoints_following/config/params.yaml`
    self.waypoint_following_node_name = "/" + rospy.get_param('~waypoint_following_node_name', 'cultureid_waypoints_following')
    self.file_waypoints_ready_topic = self.waypoint_following_node_name + "/" + rospy.get_param('~file_waypoints_ready_topic', 'file_waypoints_ready')
    self.go_to_next_waypoint_trigger_topic = self.waypoint_following_node_name + "/" + rospy.get_param('~go_to_next_waypoint_trigger_topic', 'go_to_next_waypoint_trigger')
    self.stopped_at_waypoint_topic = self.waypoint_following_node_name + "/" + rospy.get_param('~stopped_at_waypoint_topic', 'stopped_at_waypoint')

    # Create publisher to publish empty msg that triggers the robot to go to
    # THE VERY FIRST pose of the game. This pose is where the audience first
    # starts playing the game. The very first pose should be set as the pose
    # at the very top in the
    # `cultureid_waypoints_following/saved_path/xxx_pose_0` .csv file and
    # at the last column it should have a 1 marking it (the robot should
    # stop there after arriving there)
    self.goto_start_pose_pub = rospy.Publisher(self.file_waypoints_ready_topic, Empty, queue_size=1)

    # Create publisher to publish empty msg that triggers the robot to go to
    # the next init pose. Each init pose should be marked in the
    # `cultureid_waypoints_following/saved_path/xxx_pose_0` .csv file
    # by a 1 at the very last column (the robot should stop at that pose
    # and await further instructions)
    self.goto_next_pose_pub = rospy.Publisher(self.go_to_next_waypoint_trigger_topic, Empty, queue_size=1)


    rospy.sleep(2)


  ############################################################################
  def execute(self, userdata):

    # This is the `g_counter`-{st|nd|rd|th} part of the game
    userdata.g_counter = userdata.g_counter + 1

    rospy.logwarn('COUNTER = %d' % userdata.g_counter)

    empty_msg = Empty()
    if userdata.g_counter == 1: # If this is the first time go to the start pose
      self.goto_start_pose_pub.publish(empty_msg)
    else:                       # If not go to the next init pose
      self.goto_next_pose_pub.publish(empty_msg)

    # Wait for the robot to go to the designated pose
    rospy.wait_for_message(self.stopped_at_waypoint_topic, Empty)

    # Path is ready! return success and move on to the next state (FOLLOW_PATH)
    return 'success'


################################################################################
################################################################################
class ShowGui(State):

  ############################################################################
  def __init__(self):
    State.__init__(self, outcomes=['success'], input_keys=['g_counter'], output_keys=['g_counter'])
    self.ppkg = rospy.get_param('~ppkg', '/home/cultureid_user0/catkin_ws/src/cultureid_tiff22_game')
    self.gui_start_topic = rospy.get_param('~gui_start_topic', 'gui_start')
    self.gui_start_pub = rospy.Publisher(self.gui_start_topic, Empty, queue_size=1)

  ############################################################################
  def execute(self, userdata):

    empty_msg = Empty()
    self.gui_start_pub.publish(empty_msg)
    return 'success'


################################################################################
################################################################################
class ReadRFIDCard(State):

  ############################################################################
  def __init__(self):
    State.__init__(self, outcomes=['success'], input_keys=['g_counter'], output_keys=['g_counter'])
    self.read_rfid_card_alert_topic = rospy.get_param('~read_rfid_card_alert_topic', 'read_rfid_card_alert')
    self.rfid_java_exec_dir = rospy.get_param('~rfid_java_exec_dir', 'rfid_java_exec_dir')
    self.epc_answer_validity_topic  = rospy.get_param('~epc_answer_validity_topic', 'epc_answer_validity')
    self.epc_answer_validity_pub = rospy.Publisher(self.epc_answer_validity_topic, Int8, queue_size=1)

  ############################################################################
  def execute(self, userdata):

      rospy.logwarn('WAITING TO READ RFID CARD')
      rospy.wait_for_message(self.read_rfid_card_alert_topic, Empty)
      rospy.logwarn('READING RFID CARD')

      self.open_rfid_reader()

      is_answer_correct = False
      while is_answer_correct == False:
        val = self.check_rfid_answer_validity(userdata)

        if val == -1:
          rospy.loginfo('-1')
        if val == True:
          eav = Int8()
          eav.data = 1
          self.epc_answer_validity_pub.publish(eav)

          rospy.loginfo('true')
          is_answer_correct = True
        if val == False:
          eav = Int8()
          eav.data = 0
          self.epc_answer_validity_pub.publish(eav)

          rospy.loginfo('false')

        rospy.sleep(1.0)


      self.close_rfid_reader()

      return 'success'



  ############################################################################
  def open_rfid_reader(self):

    cmd = 'cd ' + self.rfid_java_exec_dir + ';' + './open_rfid_reader.sh'
    os.system(cmd)

  ############################################################################
  def close_rfid_reader(self):

    cmd = 'cd ' + self.rfid_java_exec_dir + ';' + './close_rfid_reader.sh'
    os.system(cmd)

  ############################################################################
  def check_rfid_answer_validity(self, userdata):

    # Get the correct answer
    correct_answers = self.read_file(self.rfid_java_exec_dir +'/correct_answers_epcs.txt')
    correct_answer = correct_answers[userdata.g_counter-1]
    correct_answer = correct_answer[0:len(correct_answer)-1]

    # Get the measurements
    measurements = self.read_file(self.rfid_java_exec_dir +'/results_001625143965.txt')
    if len(measurements) > 0:
      epc_most_measurements, proportion_of_measurements = self.get_epc_with_most_measurements(measurements)

      self.reset_file(self.rfid_java_exec_dir +'/results_001625143965.txt')

      if (proportion_of_measurements > 0.5) and (epc_most_measurements == correct_answer):
        return True
      else:
        return False
    else:
      return -1


  ############################################################################
  def read_file(self, file_str):
    with open(file_str,'r') as f:
      lines  = f.readlines()
      f.close()
      return lines

  ############################################################################
  def reset_file(self, file_str):
    with open(file_str,'w') as f:
      f.close()

  ############################################################################
  def get_epc_with_most_measurements(self, measurements):

    unique_epcs = []
    unique_epc_counter = []

    for m in measurements:
      m = m.split(',')
      m = m[2]
      m = m[1:]

      if (m in unique_epcs) == False:
        unique_epcs.append(m)
        unique_epc_counter.append(0)

      if (m in unique_epcs) == True:
        indx = unique_epcs.index(m)
        unique_epc_counter[indx] = unique_epc_counter[indx] + 1

    return unique_epcs[np.argmax(unique_epc_counter)], np.max(unique_epc_counter) / len(measurements)



################################################################################
################################################################################
class CheckLevelFinished(State):

  ############################################################################
  def __init__(self):
    State.__init__(self, outcomes=['iterate','end'], input_keys=['g_counter'], output_keys=['g_counter'])
    self.level_finished_alert_topic = rospy.get_param('~level_finished_alert_topic', '')

    # The number of levels in game a
    self.num_levels = 3

  ############################################################################
  def execute(self, userdata):

    rospy.wait_for_message(self.level_finished_alert_topic, Empty)

    if userdata.g_counter < self.num_levels:
      return 'iterate'
    else:
      return 'end'


################################################################################
################################################################################
class EndState(State):

  ############################################################################
  def __init__(self):
    State.__init__(self, outcomes=['game_over'], input_keys=[], output_keys=[])

    self.waypoint_following_node_name = "/" + rospy.get_param('~waypoint_following_node_name', 'cultureid_waypoints_following')
    self.stopped_at_waypoint_topic = self.waypoint_following_node_name + "/" + rospy.get_param('~stopped_at_waypoint_topic', 'stopped_at_waypoint')
    self.last_wp_reached_pub = rospy.Publisher(self.stopped_at_waypoint_topic, Empty, queue_size=1)

  ############################################################################
  def execute(self, userdata):

    rospy.sleep(1.0)
    empty_msg = Empty()
    self.last_wp_reached_pub.publish(empty_msg)

    rospy.sleep(5.0)
    rospy.signal_shutdown('game over')
    sys.exit(1)

    return 'game_over'





################################################################################
################################################################################
def main():

  # Init node
  rospy.init_node('cultureid_game_a', disable_signals=True)

  #global this_node_name
  #this_node_name = rospy.get_name().lstrip('/')

  sm = StateMachine(outcomes=['success'])
  sm.userdata.g_counter = 0

  with sm:

    StateMachine.add('GOTO_INIT_POSE', GotoInitPose(),
                       transitions={'success':'SHOW_GUI'},
                       remapping={'g_counter':'g_counter'})
    StateMachine.add('SHOW_GUI', ShowGui(),
                       transitions={'success':'READ_RFID_CARD'},
                       remapping={'g_counter':'g_counter'})
    StateMachine.add('READ_RFID_CARD', ReadRFIDCard(),
                       transitions={'success':'CHECK_LEVEL_FINISHED'},
                       remapping={'g_counter':'g_counter'})
    StateMachine.add('CHECK_LEVEL_FINISHED', CheckLevelFinished(),
                       transitions={'iterate':'GOTO_INIT_POSE', 'end':'END_STATE'},
                       remapping={'g_counter':'g_counter'})
    StateMachine.add('END_STATE', EndState(),
                       transitions={'game_over':'END_STATE'},
                       remapping={})

  outcome = sm.execute()



################################################################################
################################################################################
if __name__ == '__main__':
  main()
