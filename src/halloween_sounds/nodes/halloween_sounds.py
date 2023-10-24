#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from gtts import gTTS
import playsound
import random

def callback(data):
    state = data.data
    rospy.loginfo(rospy.get_caller_id() + "Speaking to children now because I heard %s", state)

    if state == 'intro':
        intro()
    elif state == 'outro':
        outro()

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("speech",String,callback)
    rospy.spin()

def intro():
    #  Greetings for the robot to say
    #intro = ['Hello, would you like some candy?']
    intro = ['vader_intro_1.wav', 'vader_intro_2.wav']
    #  randomly pick intro to play
    msg = random.choice(intro)
    #tts = gTTS(text=msg, lang='en')
    path = '/home/hello-robot/stretchit_halloween_candy_distributer/src/halloween_sounds/nodes/sounds_dir/' + msg
    #tts.save(path)
    playsound.playsound(path, True)

def outro():
    #  Greetings for the robot to say
    #outro = ['Have a SPOOKTACULAR Halloween!','May your candy last you at least til Christmas!','Hope your Halloween is full of more treats than tricks!','Have a boooo-tiful Halloween!',
        #'Happy Halloween to the cutest pumpkin in the patch.','Happy Halloween and enjoy that sugar rush tonight!']
    outro = ['vader_outro_1.wav', 'vader_outro_2.wav']
    #  randomly pick outro to play
    msg = random.choice(outro)
    #tts = gTTS(text=msg, lang='en')
    path = '/home/hello-robot/stretchit_halloween_candy_distributer/src/halloween_sounds/nodes/sounds_dir/' + msg
    #tts.save(path)
    playsound.playsound(path, True)

if __name__ == '__main__':
        listener()