#!/usr/bin/env python

import sys
import threading
import os
import time
import rospy
from std_msgs.msg import Empty
from std_msgs.msg import Int8
import Tkinter
import numpy as np
import time
from subprocess import call
from functools import partial
from sys import argv

# Sending goals etc
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseFeedback, MoveBaseResult



class GuiGameB():

################################################################################
# constructor
################################################################################
  def __init__(self):

    self.root = Tkinter.Tk()
    self.root.attributes('-fullscreen',True)
    self.q = 0
    self.dir_media = rospy.get_param('~dir_media', '')
    self.dir_scripts = rospy.get_param('~dir_scripts', '')
    self.poses = rospy.get_param('~poses', '')


    if self.dir_media == '':
      print('[cultureid_games_tif22] gui_game_b_node dir_media not set; aborting')
      return

    if self.dir_scripts == '':
      print('[cultureid_games_tif22] gui_game_b_node dir_scripts not set; aborting')
      return

    if self.poses == '':
      print('[cultureid_games_tif22] gui_game_b_node poses not set; aborting')
      return

    # read tour questions
    self.Qm_vec = []
    with open(self.dir_media+'/Qm.txt','r') as f:
      lines = f.readlines()
      f.close()
    for line in lines:
      self.Qm_vec.append(line[0:len(line)-1])

    # read tour choices
    self.Am_vec = []
    with open(self.dir_media+'/Am.txt','r') as f:
      lines = f.readlines()
      f.close()
    for line,num in zip(lines,range(len(lines))):
      self.Am_vec.append([])
      line = line.split(',')
      c1 = 1
      for item in line:
        if c1 != len(line):
          self.Am_vec[-1].append(item)
        else:
          self.Am_vec[-1].append(item[0:len(item)-1])
        c1 = c1+1

    # Move base action server
    self.action_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
    #rospy.loginfo("Waiting for move base server")
    #self.action_client.wait_for_server()



    # Go to start pose (you are the host)
    self.goto_pose(0)


    # Seems that the mainloop should be placed here; otherwise throws error
    rospy.logwarn('root mainloop')
    self.root.mainloop()


################################################################################
  def goto_pose(self, num):

    # Orientation is quaternion
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.pose.position.x = self.poses[num][0]
    goal.target_pose.pose.position.y = self.poses[num][1]
    goal.target_pose.pose.position.z = self.poses[num][2]
    goal.target_pose.pose.orientation.x = self.poses[num][3]
    goal.target_pose.pose.orientation.y = self.poses[num][4]
    goal.target_pose.pose.orientation.z = self.poses[num][5]
    goal.target_pose.pose.orientation.w = self.poses[num][6]

    # Maybe clear costmaps here TODO
    rospy.logwarn('sending goal and waiting for result')
    #self.action_client.send_goal(goal)
    #self.action_client.wait_for_result()

    rospy.logwarn('am at pose %d' %num)

    if (num == 0):
      self.start_game()
    else:
      self.start_video(num)

    return



################################################################################
  def get_x_y_dims(self,desiredNum):

    side1 = int(desiredNum/int(desiredNum**0.5))
    side2 = int(desiredNum/side1)

    sideSmall = np.min([side1,side2])
    sideLarge = np.max([side1,side2])

    if sideSmall*sideLarge < desiredNum:
      sideSmall = sideSmall+1

    side1 = np.min([sideSmall,sideLarge])
    side2 = np.max([sideSmall,sideLarge])

    return side1,side2



################################################################################
  def kill_root(self):
    self.root.destroy()
    rospy.signal_shutdown('game over')
    os._exit(os.EX_OK)


################################################################################
  def playVid(self, num):

    rospy.logwarn('playVid')

    self.q = num

    # clean window
    for frames in self.root.winfo_children():
      frames.destroy()

    call(['vlc','--no-repeat','--fullscreen','--play-and-exit', self.dir_media + '/' + str(self.q)+'.mp4'])

    rospy.sleep(10.0)

    # Go to the beginning
    self.goto_pose(0)


################################################################################
  def showQm(self):

    rospy.logwarn('showQm')

    # clean window
    for frames in self.root.winfo_children():
      frames.destroy()

    # new canvas
    canvas = Tkinter.Canvas(self.root)
    canvas.configure(bg='red')
    canvas.pack(fill=Tkinter.BOTH,expand=True)

    # to frame panw sto opoio 8a einai ta koumpia
    frame = Tkinter.Frame(self.root,bg='grey')
    frame.place(relwidth=0.95,relheight=0.95,relx=0.025,rely=0.025)

    # ta koumpia tou para8urou erwthsh

    this_Qm = self.Qm_vec[0]

    buttonVec = []
    buttonText = []

    QButton = Tkinter.Button(frame,text='???',fg='black',bg='white')
    buttonVec.append(QButton)
    buttonText.append(this_Qm)

    xNum = 1
    yNum = len(buttonVec)

    xEff = 1.0
    yEff = 0.4

    GP = 0.05

    xWithGuard = xEff/xNum
    xG = GP*xWithGuard
    xB = xWithGuard-xG

    yWithGuard = yEff/yNum
    yG = GP*yWithGuard
    yB = yWithGuard-yG

    counter = 0
    for xx in range(xNum):
      for yy in range(yNum):
        thisX = xG/2+xx*xWithGuard
        thisY = yG/2+yy*yWithGuard

        buttonVec[counter].place(relx=thisX,rely=thisY,relheight=yB,relwidth=xB)
        buttonVec[counter].config(text=buttonText[counter])
        buttonVec[counter].update()

        thisWidth = buttonVec[counter].winfo_width()
        thisHeight = buttonVec[counter].winfo_height()
        buttonVec[counter].update()

        counter = counter+1

    # ta koumpia tou para8urou apanthseis

    this_Am = self.Am_vec[0]

    buttonVec = []
    buttonText = []

    for answer,num in zip(this_Am,range(len(this_Am))):
      this_butt = Tkinter.Button(frame,text='???',fg='black',bg='white',command=partial(self.goto_pose,num+1))
      buttonVec.append(this_butt)
      buttonText.append(answer)

    xNum,yNum = self.get_x_y_dims(len(buttonVec))

    #xNum = 1
    #yNum = len(buttonVec)

    xEff = 1.0
    yEff = 0.6

    GP = 0.05

    xWithGuard = xEff/xNum
    xG = GP*xWithGuard
    xB = xWithGuard-xG

    yWithGuard = yEff/yNum
    yG = GP*yWithGuard
    yB = yWithGuard-yG

    counter = 0
    for xx in range(xNum):
      for yy in range(yNum):

        if counter < len(buttonVec):
          thisX = xG/2+xx*xWithGuard
          thisY = yG/2+yy*yWithGuard+1-yEff

          buttonVec[counter].place(relx=thisX,rely=thisY,relheight=yB,relwidth=xB)
          buttonVec[counter].config(text=buttonText[counter])
          buttonVec[counter].update()

          thisWidth = buttonVec[counter].winfo_width()
          thisHeight = buttonVec[counter].winfo_height()
          buttonVec[counter].update()

        counter = counter+1



################################################################################
  def start_game(self):
    rospy.logwarn('start_game')

    # clean window
    for frames in self.root.winfo_children():
      frames.destroy()


    # new canvas
    canvas = Tkinter.Canvas(self.root)
    canvas.configure(bg='red')
    canvas.pack(fill=Tkinter.BOTH,expand=True)

    # to frame panw sto opoio 8a einai ta koumpia
    frame = Tkinter.Frame(self.root,bg='grey')
    frame.place(relwidth=0.95,relheight=0.95,relx=0.025,rely=0.025)

    # ta koumpia tou para8urou
    buttonVec = []
    buttonText = []

    waiting_for_pose = Tkinter.Button(frame,text='???',fg='black',bg='white',command=self.showQm)
    buttonVec.append(waiting_for_pose)
    buttonText.append('START TOUR')

    xNum = 1
    yNum = len(buttonVec)

    xEff = 1
    yEff = 1

    GP = 0.05

    xWithGuard = xEff/xNum
    xG = GP*xWithGuard
    xB = xWithGuard-xG

    yWithGuard = yEff/yNum
    yG = GP*yWithGuard
    yB = yWithGuard-yG

    counter = 0
    for xx in range(xNum):
      for yy in range(yNum):
        thisX = xG/2+xx*xWithGuard
        thisY = yG/2+yy*yWithGuard

        buttonVec[counter].place(relx=thisX,rely=thisY,relheight=yB,relwidth=xB)
        buttonVec[counter].config(text=buttonText[counter])
        buttonVec[counter].update()

        thisWidth = buttonVec[counter].winfo_width()
        thisHeight = buttonVec[counter].winfo_height()
        buttonVec[counter].update()

        counter = counter+1
    '''
    ## wait for state change
    ch_bool = wait_state_change()
    while ch_bool == False:
      print(ch_bool)
      time.sleep(1)
      ch_bool = wait_state_change()
    '''



################################################################################
  def start_video(self, num):
    rospy.logwarn('start_video')

    # clean window
    for frames in self.root.winfo_children():
      frames.destroy()

    # new canvas
    canvas = Tkinter.Canvas(self.root)
    canvas.configure(bg='red')
    canvas.pack(fill=Tkinter.BOTH,expand=True)

    # to frame panw sto opoio 8a einai ta koumpia
    frame = Tkinter.Frame(self.root,bg='grey')
    frame.place(relwidth=0.95,relheight=0.95,relx=0.025,rely=0.025)

    # ta koumpia tou para8urou
    buttonVec = []
    buttonText = []

    waiting_for_pose = Tkinter.Button(frame,text='???',fg='black',bg='white',command=partial(self.playVid,num))
    buttonVec.append(waiting_for_pose)
    buttonText.append('START VIDEO')

    xNum = 1
    yNum = len(buttonVec)

    xEff = 1
    yEff = 1

    GP = 0.05

    xWithGuard = xEff/xNum
    xG = GP*xWithGuard
    xB = xWithGuard-xG

    yWithGuard = yEff/yNum
    yG = GP*yWithGuard
    yB = yWithGuard-yG

    counter = 0
    for xx in range(xNum):
      for yy in range(yNum):
        thisX = xG/2+xx*xWithGuard
        thisY = yG/2+yy*yWithGuard

        buttonVec[counter].place(relx=thisX,rely=thisY,relheight=yB,relwidth=xB)
        buttonVec[counter].config(text=buttonText[counter])
        buttonVec[counter].update()

        thisWidth = buttonVec[counter].winfo_width()
        thisHeight = buttonVec[counter].winfo_height()
        buttonVec[counter].update()

        counter = counter+1








################################################################################
# main
################################################################################
if __name__ == '__main__':

  rospy.init_node('gui_game_b_node')

  try:
    gga = GuiGameB()
    rospy.spin()

  except rospy.ROSInterruptException:
    print("SHUTTING DOWN")
    pass
