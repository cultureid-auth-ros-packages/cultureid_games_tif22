#!/usr/bin/env python

import sys
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



class GuiGameA():

################################################################################
# constructor
################################################################################
  def __init__(self):

    self.root = Tkinter.Tk()
    self.root.attributes('-fullscreen',True)
    self.q = 0
    self.total_errors = 0
    self.time_start = time.time()
    self.dir_media = rospy.get_param('~dir_media', '')
    self.dir_scripts = rospy.get_param('~dir_scripts', '')
    self.read_rfid_card_alert_topic = rospy.get_param('~read_rfid_card_alert_topic', '')
    self.level_finished_alert_topic = rospy.get_param('~level_finished_alert_topic', '')
    self.gui_start_topic = rospy.get_param('~gui_start_topic', '')
    self.epc_answer_validity_topic  = rospy.get_param('~epc_answer_validity_topic', 'epc_answer_validity')

    self.waypoint_following_node_name = "/" + rospy.get_param('~waypoint_following_node_name', 'cultureid_waypoints_following')
    self.stopped_at_waypoint_topic = self.waypoint_following_node_name + "/" + rospy.get_param('~stopped_at_waypoint_topic', 'stopped_at_waypoint')

    if self.dir_media == '':
      print('[cultureid_tiff22_game] gui_game_a_node dir_media not set; aborting')
      return

    if self.dir_scripts == '':
      print('[cultureid_tiff22_game] gui_game_a_node dir_scripts not set; aborting')
      return

    if self.read_rfid_card_alert_topic == '':
      print('[cultureid_tiff22_game] gui_game_a_node read_rfid_card_alert_topic not set; aborting')
      return

    if self.level_finished_alert_topic == '':
      print('[cultureid_tiff22_game] gui_game_a_node level_finished_alert_topic not set; aborting')
      return

    if self.gui_start_topic == '':
      print('[cultureid_tiff22_game] gui_game_a_node gui_start_topic not set; aborting')
      return

    # Subscribes to the topic which alerts the gui to start; may change itf
    self.gui_start_sub = rospy.Subscriber(self.gui_start_topic, Empty, self.gui_start_callback)

    # Publishes a msg that alerts the fsm that the rfid reader should start
    # reading for the desired EPC
    self.read_rfid_card_alert_pub = rospy.Publisher(self.read_rfid_card_alert_topic, Empty, queue_size=1, latch=True)

    # Publishes a msg that alerts the fsm that this level (self.q) is finished
    self.level_finished_alert_pub = rospy.Publisher(self.level_finished_alert_topic, Empty, queue_size=1, latch=True)



    # read multiple choice questions
    self.Qm_vec = []
    with open(self.dir_media+'/Qm.txt','r') as f:
      lines = f.readlines()
      f.close()
    for line in lines:
      self.Qm_vec.append(line[0:len(line)-1])

    # read multiple choice answers
    self.Am_vec = []
    self.Am_cor = []
    with open(self.dir_media+'/Am.txt','r') as f:
      lines = f.readlines()
      f.close()
    for line,num in zip(lines,range(len(lines))):
      if num % 2 == 0:
        self.Am_vec.append([])
        line = line.split(',')
        c1 = 1
        for item in line:
          if c1 != len(line):
            self.Am_vec[-1].append(item)
          else:
            self.Am_vec[-1].append(item[0:len(item)-1])
          c1 = c1+1
      else:
        self.Am_cor.append(int(line))

    # read card-key questions
    self.Qc_vec = []
    with open(self.dir_media+'/Qc.txt','r') as f:
      lines = f.readlines()
      f.close()
    for line in lines:
      self.Qc_vec.append(line[0:len(line)-1])

    # read card-key tag info
    self.tag_names = []
    self.tag_epcs= []
    with open(self.dir_media+'/tag_info.txt','r') as f:
      lines = f.readlines()
      f.close()
    for line in lines:
      line = line.split(',')
      self.tag_names.append(line[0])
      self.tag_epcs.append(line[1][0:len(line[1])-1])

    # Seems that the mainloop should be placed here; otherwise throws error
    self.root.mainloop()



################################################################################
  def gui_start_callback(self, msg):

    rospy.logwarn('[cultureid_tiff22_game] gui_start_callback')

    if self.q == 0:
      self.start_game()
    elif self.q < len(self.tag_epcs):
      self.continue_game()
    else:
      self.end_game()


################################################################################
  def continue_game(self):
    self.showStartButton()

################################################################################
  def end_game(self):

    self.root.destroy()
    sys.exit(1)
    return


################################################################################
  def check_correct_answer(self,num):

    if num == self.Am_cor[self.q]:
      self.correct_answer()

      self.q = self.q + 1

      call(['cvlc', '--no-repeat','--play-and-exit','/home/cultureid_user0/Desktop/oh_yeah.mp3'])

      # Let the fsm know that the rfid reader should start execution
      rospy.logwarn('[cultureid_tiff22_game] Sending msg to alert for level finish')
      empty_msg = Empty()
      self.level_finished_alert_pub.publish(empty_msg)


      # Wait until the robot moves to the next initial pose
      rospy.logwarn('waiting to receive stop at wp signal')
      rospy.wait_for_message(self.stopped_at_waypoint_topic, Empty)
      rospy.logwarn('received stop at wp signal')

    else:
      self.wrong_answer()

################################################################################
  def wrong_answer(self):

    # increase total erros
    self.total_errors = self.total_errors+1

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

    playButton = Tkinter.Button(frame,text='???',fg='black',bg='white', command=self.showQm)
    buttonVec.append(playButton)
    buttonText.append('wrong! haha loser')

    xNum = 1
    yNum = len(buttonVec)

    xEff = 1.0
    yEff = 1.0

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
  def kill_root(self):
    self.root.destroy()
    os._exit(os.EX_OK)


################################################################################
  def showStats(self):

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

    # wrong answers button
    wrongAnsBut = Tkinter.Button(frame,text='???',fg='black',bg='white')
    buttonVec.append(wrongAnsBut)
    buttonText.append('Wrong Answers: '+str(self.total_errors))

    # total time button
    totalTimeBut = Tkinter.Button(frame,text='???',fg='black',bg='white')
    buttonVec.append(totalTimeBut)
    total_time = time.time()-self.time_start
    buttonText.append('Game Time: '+str(total_time)+' s')

    # exit button
    exitBut = Tkinter.Button(frame,text='???',fg='black',bg='white',command=self.kill_root)
    buttonVec.append(exitBut)
    buttonText.append('Quit :(')

    xNum = 1
    yNum = len(buttonVec)

    xEff = 1.0
    yEff = 1.0

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
  def correct_answer(self):

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

    playButton = Tkinter.Button(frame,text='???',fg='black',bg='white')
    buttonVec.append(playButton)
    buttonText.append('Bravo! Correct Answer!')

    if self.q == len(self.tag_epcs)-1:
      playButton.config(text='Emfanish apotelematwn')
      playButton.config(command=self.showStats)
      playButton.update()

    xNum = 1
    yNum = len(buttonVec)

    xEff = 1.0
    yEff = 1.0

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
  def showStartButton(self):

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

    playButton = Tkinter.Button(frame,text='???',fg='black',bg='white',command=self.playVid)
    buttonVec.append(playButton)
    buttonText.append('play_video')

    xNum = 1
    yNum = len(buttonVec)

    xEff = 1.0
    yEff = 0.5

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
        thisY = yG/2+yy*yWithGuard + 1.0-yEff

        buttonVec[counter].place(relx=thisX,rely=thisY,relheight=yB,relwidth=xB)
        buttonVec[counter].config(text=buttonText[counter])
        buttonVec[counter].update()

        thisWidth = buttonVec[counter].winfo_width()
        thisHeight = buttonVec[counter].winfo_height()
        buttonVec[counter].update()

        counter = counter+1


################################################################################
  def wait_state_change(self):
    with open(self.dir_scripts+'/change_state.txt','r') as f:
      lines  = f.readlines()
      f.close()

    change_state = int(lines[0])

    if change_state == 1:
      with open(self.dir_scripts+'/change_state.txt','w') as f:
        f.write('0')
        f.close()
      return True
    else:
      return False

################################################################################
  def playVid(self):

    # clean window
    for frames in self.root.winfo_children():
      frames.destroy()

    call(['vlc','--no-repeat','--fullscreen','--play-and-exit', self.dir_media + '/' + str(self.q)+'.mp4'])

    self.showQc()

################################################################################
  def showQc(self):

    this_Qc = self.Qc_vec[self.q]


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

    QButton = Tkinter.Button(frame,text='???',fg='black',bg='white')
    buttonVec.append(QButton)
    buttonText.append(this_Qc)


    xNum = 1
    yNum = len(buttonVec)

    xEff = 1.0
    yEff = 0.5

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
        thisY = yG/2+yy*yWithGuard + 1.0-yEff

        buttonVec[counter].place(relx=thisX,rely=thisY,relheight=yB,relwidth=xB)
        buttonVec[counter].config(text=buttonText[counter])
        buttonVec[counter].update()

        thisWidth = buttonVec[counter].winfo_width()
        thisHeight = buttonVec[counter].winfo_height()
        buttonVec[counter].update()

        counter = counter+1


    # Let the fsm know that the rfid reader should start execution
    rospy.logwarn('[cultureid_tiff22_game] Sending msg to alert for RFID reading')
    empty_msg = Empty()
    self.read_rfid_card_alert_pub.publish(empty_msg)


    # wait to be alerted about the outcome of the question
    correct_answer = False
    while correct_answer == False:
      msg = rospy.wait_for_message(self.epc_answer_validity_topic, Int8)
      if msg.data == 1:
        QButton.config(text='CORRECT!')
        QButton.config(command=self.showQm)
        QButton.update()
        rospy.sleep(2.0)
        correct_answer = True
      if msg.data == 0:
        self.total_errors = self.total_errors+1
        QButton.config(text='WRONG :(')
        QButton.update()




################################################################################
  def showQm(self):

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

    this_Qm = self.Qm_vec[self.q]

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

    this_Am = self.Am_vec[self.q]

    buttonVec = []
    buttonText = []

    for answer,num in zip(this_Am,range(len(this_Am))):
      this_butt = Tkinter.Button(frame,text='???',fg='black',bg='white',command=partial(self.check_correct_answer,num))
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

    waiting_for_pose = Tkinter.Button(frame,text='???',fg='black',bg='white',command=self.showStartButton)
    buttonVec.append(waiting_for_pose)
    buttonText.append('START GAME')

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
# main
################################################################################
if __name__ == '__main__':

  rospy.init_node('gui_game_a_node')

  try:
    gga = GuiGameA()
    rospy.spin()

  except rospy.ROSInterruptException:
    print("SHUTTING DOWN")
    pass
