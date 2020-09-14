#!/usr/bin/env python3

# this script publishes keystrokes from the laptop to a topic
# called 'keys_pressed' from the node 'key_publisher'

import rospy
from std_msgs.msg import String

from pynput.keyboard import Key, Listener, Controller
from pynput import keyboard

# The key combination to check
COMBINATION1 = {keyboard.Key.up, keyboard.Key.right}
COMBINATION2 = {keyboard.Key.up, keyboard.Key.left}
COMBINATION3 = {keyboard.Key.down, keyboard.Key.right}
COMBINATION4 = {keyboard.Key.down, keyboard.Key.left}
COMBINATION5 = {keyboard.Key.down}
COMBINATION6 = {keyboard.Key.left}
COMBINATION7 = {keyboard.Key.right}
COMBINATION8 = {keyboard.Key.up}
controller = Controller()

# the ros node
rospy.init_node('key_publisher', anonymous=True)

# the topic publishing to
pub = rospy.Publisher('keys_pressed', String, queue_size=10)

rate = rospy.Rate(10)

# create an empty set for the currently pressed keys to be stored in
keys_pressed = set()

# when a key is pressed down, add it to the set, determine the combi pressed
def on_press(key):
    keys_pressed.add(key)
    if all(k in keys_pressed for k in COMBINATION1):
        pub.publish("UR")
        print('UP RIGHT')
    elif all(k in keys_pressed for k in COMBINATION2):
        pub.publish("UL")
        print('UP LEFT')
    elif all(k in keys_pressed for k in COMBINATION3):
        pub.publish("DR")
        print('DOWN RIGHT')
    elif all(k in keys_pressed for k in COMBINATION4):
        pub.publish("DL")
        print('DOWN LEFT')
    elif all(k in keys_pressed for k in COMBINATION5):
        pub.publish("D")
        print('DOWN')
    elif all(k in keys_pressed for k in COMBINATION6):
        pub.publish("L")
        print('LEFT')
    elif all(k in keys_pressed for k in COMBINATION7):
        pub.publish("R")
        print('RIGHT')
    elif all(k in keys_pressed for k in COMBINATION8):
        pub.publish("U")
        print('UP')
    if key == keyboard.Key.esc: # stop the topic with the esc key
        listener.stop()


def on_release(key): # when a key is released, publish "pause", remove it from the set
    try:
        if key == keyboard.Key.up or key == keyboard.Key.down:
            pub.publish("PD") # pause turn
        elif key == keyboard.Key.right or key == keyboard.Key.left:
            pub.publish("PT") # pause drive
        keys_pressed.remove(key) # remove key from set

        # when release a key, the other key that is still pressed is not detected 
        # for some reason, so "fake press" it down
        key_left = keys_pressed.pop()
        keys_pressed.add(key_left)
        controller.press(key_left)
    except KeyError:
        pass
    

if __name__ == '__main__':
    try:
      with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
        listener.join()
    except rospy.ROSInterruptException:
      pass
