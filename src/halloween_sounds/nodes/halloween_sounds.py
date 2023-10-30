#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from gtts import gTTS
import playsound
import random

def callback(data):
    state = data.data
    rospy.loginfo(rospy.get_caller_id() + "Speaking to people now because I heard %s", state)

    if state == 'intro':
        intro()
    elif state == 'outro':
        outro()
    elif state == 'vader':
        darthvader()
    elif state == 'minion':
        minions()    
    elif state == "beep":
        beep()

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("speech",String,callback)
    rospy.spin()

def intro():
    #  Greetings for the robot to say
    intro = ['intro_1.wav']
    #  randomly pick intro to play
    msg = random.choice(intro)
    path = '/home/hello-robot/stretchit_halloween_candy_distributer/src/halloween_sounds/nodes/sounds_dir/' + msg
    rospy.loginfo('Playing intro...')
    playsound.playsound(path, True)

    #intro = ['Hello, would you like some candy?']
    #tts = gTTS(text=msg, lang='en')
    #tts.save(path)

def outro():
    #  Greetings for the robot to say
    outro = ['outro_1.wav', 'outro_2.wav', 'outro_3.wav', 'outro_4.wav']
    #  randomly pick outro to play
    msg = random.choice(outro)
    path = '/home/hello-robot/stretchit_halloween_candy_distributer/src/halloween_sounds/nodes/sounds_dir/' + msg
    rospy.loginfo('Playing outro...')
    playsound.playsound(path, True)

    #outro = ['Have a SPOOKTACULAR Halloween!','May your candy last you at least til Christmas!','Hope your Halloween is full of more treats than tricks!','Have a boooo-tiful Halloween!',
    #'Happy Halloween to the cutest pumpkin in the patch.','Happy Halloween and enjoy that sugar rush tonight!']
    #tts = gTTS(text=msg, lang='en')
    #tts.save(path)

def darthvader():
    #  Random darthvader sayings
    darth = ['vader_1.wav', 'vader_2.wav', 'vader_3.wav', 'vader_4.wav', 'vader_5.wav', 'vader_6.wav', 'vader_7.wav', 'vader_8.wav', 'vader_9.wav', 'vader_10.wav', 'vader_11.wav', 'vader_12.wav']
    #  randomly pick audio to play
    msg = random.choice(darth)
    path = '/home/hello-robot/stretchit_halloween_candy_distributer/src/halloween_sounds/nodes/sounds_dir/' + msg
    rospy.loginfo('Playing darth...')
    playsound.playsound(path, True)

    #tts = gTTS(text=msg, lang='en')
    #tts.save(path)
def minions():
    #  Random minions sayings
    minion = ['minion_1.mp3', 'minion_2.mp3','minion_3.mp3','minion_4.mp3','minion_5.mp3','minion_6.mp3','minion_7.mp3','minion_8.mp3']
    #  randomly pick audio to play
    msg = random.choice(minion)
    path = '/home/hello-robot/stretchit_halloween_candy_distributer/src/halloween_sounds/nodes/sounds_dir/' + msg
    rospy.loginfo('Playing minion...')
    playsound.playsound(path, True)

def beep():
    path = '/home/hello-robot/stretchit_halloween_candy_distributer/src/halloween_sounds/nodes/sounds_dir/beep.mp3'
    rospy.loginfo('Playing beep...')
    playsound.playsound(path, True)
if __name__ == '__main__':
        listener()