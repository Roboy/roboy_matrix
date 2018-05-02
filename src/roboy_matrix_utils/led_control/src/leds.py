#!/usr/bin/env python

import rospy
import time
import numpy
import random
from roboy_communication_control.msg import ControlLeds
from std_msgs.msg import Empty

class MatrixLeds(object):
    """docstring for MatrixLeds"""
    def __init__(self):
        super(MatrixLeds, self).__init__()
        self.run = True
        self.channels = 4 # red green white blue
        self.leds_num = 36
        self.mode=0

    def write_pixels(self,pixels):
        # image is a list of size 4*36
        with open('/dev/matrixio_everloop','wb') as bin_file:
            bin_file.write(bytearray(pixels))
        bin_file.close()

        
    def dimming_puls(self, duration=0):
        # mode 1
        # dims in & out changing colors
        brightness = 50
        half_brightness = 0#int(100 / 2)
        # leds_num = 36
        pixels  = [0, 0, 0, half_brightness] * self.leds_num
        count = 0
        d = 1
        pos = 0
        color = [0,0,0,half_brightness]
        start = time.time()
        while self.run and self.mode==1:
            if (duration!=0 and time.time()-start>duration):
                break
            #color = [0,0,0,half_brightness]
            pixels  = color * self.leds_num
            self.write_pixels(pixels)
            # self.show(pixels)
            time.sleep(0.02)
            if (count!=1 and (count-1)%brightness==0):
                d = -d
            if((count-1)%(2*brightness)==0):
            #    print "CHANGED COLOR"
                if (pos==3):
                    pos=0
                else:
                    pos += 1
            half_brightness += d
            count += abs(d)
            color = [0]*4
            color[pos] = half_brightness
            # pixels = pixels[-2:] + pixels[:-2]

    def tail_clock(self, duration=0):
        # mode 2
        brightness = 3
        tail = 30
        led = 0
        #print "duration: ",duration
        start = time.time()
        while self.run and self.mode==2:
            if (duration!=0 and time.time()-start > duration):
                break
            intensity = brightness
            pixels = [0] * self.channels * self.leds_num
            if (led > 35 or led < 0):
                led=0
            for l in range(led-tail,led):
                intensity += 4
                pixels[l*self.channels+3]=intensity
            self.write_pixels(pixels)
            led +=1
            time.sleep(0.02)

    def set_color(self, red, green, blue, white):
        color_array = []
        for x in range(0,35):
            color_array += [red, green, blue, white]
        self.write_pixels(color_array)

    def turn_off(self):
        self.write_pixels([0]*self.channels*self.leds_num)

def mode_callback(msg):
    leds.run = True
    if (msg.mode==0):
        leds.mode=0
        leds.turn_off()
    elif (msg.mode==1):
        leds.mode=1
        leds.dimming_puls(msg.duration)
    elif (msg.mode==2):
        leds.mode=2
        leds.tail_clock(msg.duration)

def off_callback(msg):
    leds.run = False
    leds.turn_off()

def freeze_callback(msg):
    leds.run = False
    leds.mode=-1
    leds.set_color(0,0,0,15)
    
def led_listener():
    rospy.init_node('roboy_led_control')
    rospy.Subscriber("/roboy/control/matrix/leds/mode", ControlLeds, mode_callback)
    rospy.Subscriber("/roboy/control/matrix/leds/off", Empty, off_callback)
    rospy.Subscriber("/roboy/control/matrix/leds/freeze", Empty, freeze_callback)
    leds.mode=1
    leds.dimming_puls(2)
    rospy.spin()

        

if __name__ == '__main__':
    global leds
    leds = MatrixLeds()
    led_listener()
    

