#!/usr/bin/env python

import threading
import os
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
# Deprecated; does nothing but pass to the next state
################################################################################
class GotoInitPose(State):

  ############################################################################
  def __init__(self):
    State.__init__(self, outcomes=['success'], input_keys=['g_counter'], output_keys=['g_counter'])


  ############################################################################
  def execute(self, userdata):

    # This is the `g_counter`-{st|nd|rd|th} part of the game
    userdata.g_counter = userdata.g_counter + 1
    #rospy.logwarn('[cultureid_tiff22; GotoInitPose::execute] COUNTER = %d' % userdata.g_counter)

    return 'success'


################################################################################
################################################################################
class ShowGui(State):

  ############################################################################
  def __init__(self):
    State.__init__(self, outcomes=['success'], input_keys=['g_counter'], output_keys=['g_counter'])
    self.ppkg = rospy.get_param('~ppkg', '/home/cultureid_user0/catkin_ws/src/cultureid_tiff22_game')
    self.gui_start_topic = rospy.get_param('~gui_start_topic', 'gui_start')
    self.gui_start_pub = rospy.Publisher(self.gui_start_topic, Empty, queue_size=1, latch=True)

  ############################################################################
  def execute(self, userdata):

    #rospy.logwarn('[cultureid_tiff22; ShowGui::execute] Sending gui_start_msg')
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
    self.epc_answer_validity_pub = rospy.Publisher(self.epc_answer_validity_topic, Int8, queue_size=1, latch=True)

  ############################################################################
  def execute(self, userdata):

      rospy.logwarn('[cultureid_tiff22; ReadRFIDCard::execute] WAITING TO READ RFID CARD')
      rospy.wait_for_message(self.read_rfid_card_alert_topic, Empty)
      rospy.logwarn('[cultureid_tiff22; ReadRFIDCard::execute] READING RFID CARD')

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
    self.num_levels = 1

  ############################################################################
  def execute(self, userdata):

    rospy.wait_for_message(self.level_finished_alert_topic, Empty)

    rospy.logwarn('COUNTER = %d' % userdata.g_counter)

    if userdata.g_counter < self.num_levels:
      return 'iterate'
    else:
      return 'end'


################################################################################
################################################################################
class EndState(State):

  ############################################################################
  def __init__(self):
    State.__init__(self, outcomes=[], input_keys=[], output_keys=[])

    self.waypoint_following_node_name = "/" + rospy.get_param('~waypoint_following_node_name', 'cultureid_waypoints_following')
    self.stopped_at_waypoint_topic = self.waypoint_following_node_name + "/" + rospy.get_param('~stopped_at_waypoint_topic', 'stopped_at_waypoint')
    self.last_wp_reached_pub = rospy.Publisher(self.stopped_at_waypoint_topic, Empty, queue_size=10, latch=True)

  ############################################################################
  def execute(self, userdata):

    rospy.sleep(1.0)
    empty_msg = Empty()
    self.last_wp_reached_pub.publish(empty_msg)

    rospy.sleep(3.0)
    rospy.signal_shutdown('game over')
    os._exit(os.EX_OK)

    return


################################################################################
################################################################################
def main():

  # Init node
  rospy.init_node('cultureid_tiff_game_a')

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
                       transitions={},
                       remapping={})


  outcome = sm.execute()



################################################################################
################################################################################
if __name__ == '__main__':
  main()
