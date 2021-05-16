#!/usr/bin/env python
import rospy
import sys
import tty
import termios
import threading
from pocketsphinx import LiveSpeech
import rospkg
from std_msgs.msg import String

rospack = rospkg.RosPack()
key_path = rospack.get_path('rhodent_interface')+'/config/key.list'

voice_cmd_publisher = rospy.Publisher('/rhodent/room_cmd', String, queue_size=1)

speech = LiveSpeech(lm = False, kws = key_path, verbose=False, no_search=False, full_utt=False, buffer_size=2048, sampling_rate=16000)

# def parse_voice_data(words)

unique_word = ['living','office','dining','corridor','kitchen','bed','store']
match_word = ['living_room','office','dining','corridor','kitchen','bed_room','storage']

rospy.init_node("voice_command_parser")

send = 0

def getchar(): #codeforester stackoverflow
    fd = sys.stdin.fileno()
    attr = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSANOW, attr)

def listen_thread():
    global send
    while (1):
        if getchar()==' ':
            send = 1
        else:
            send = 0

job1 = threading.Thread(target=listen_thread)
job1.start()

for data in speech:
    words = str(data)
    c = 0
    for n, i in enumerate(unique_word):
        if i in words: 
            c+=1
            matchidx = n 
    if c != 1: 
        rospy.logwarn('No match or cannot decide.')
        continue
    rospy.loginfo('Matched: '+match_word[matchidx])
    
    if send == 1: 
        voice_cmd_publisher.publish(match_word[matchidx])
        rospy.loginfo("Command Sent!")
        send = 0


