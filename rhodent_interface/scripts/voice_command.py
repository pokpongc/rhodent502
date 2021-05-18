#!/usr/bin/env python
import rospy
import pygame
from pocketsphinx import LiveSpeech
import rospkg
from std_msgs.msg import String

rospack = rospkg.RosPack()
key_path = rospack.get_path('rhodent_interface')+'/config/key.list'

voice_cmd_publisher = rospy.Publisher('/rhodent/room_cmd', String, queue_size=1)

speech = LiveSpeech(lm = False, kws = key_path, verbose=False, no_search=False, full_utt=False, buffer_size=2048, sampling_rate=16000) #Initialize the speech recognizer

unique_word = ['living','office','dining','corridor','kitchen','bed','store']
match_word = ['living_room','office','dining','corridor','kitchen','bed_room','storage'] #publishing word

rospy.init_node("voice_command_parser")
pygame.init()
pygame.display.set_caption('Press to Talk')
bg = pygame.image.load(rospack.get_path('rhodent_interface')+'/media/mic_pic.png')
screen = pygame.display.set_mode((225, 225))

screen.blit(bg, (0, 0))
pygame.display.update()

for data in speech:
    words = str(data)
    c = 0
    for n, i in enumerate(unique_word): #check if matched words are unique according to the list
        if i in words: 
            c+=1
            matchidx = n 
    if c != 1:  
        rospy.logwarn('No match or cannot decide.') 
        continue #go to next dataset
    rospy.loginfo('Matched: '+match_word[matchidx])
    event = pygame.event.get()
    if len(event) > 0:
        # print (str(event))
        if "MouseButtonDown" in str(event): 
            voice_cmd_publisher.publish(match_word[matchidx])
            rospy.loginfo("Command Sent!")
            while "MouseButtonUp" not in str(pygame.event.get()): pass #wait for mouse release

