import os
import RPi.GPIO as GPIO
import time

sel1 = 17
sel2 = 27 
sel3 = 22
GPIO.setmode(GPIO.BCM)
GPIO.setup(sel1,GPIO.OUT)
GPIO.setup(sel2,GPIO.OUT)
GPIO.setup(sel3,GPIO.OUT)

def cam1():
    GPIO.output(sel1,GPIO.LOW)
    GPIO.output(sel2,GPIO.LOW)
    GPIO.output(sel3,GPIO.LOW)
def cam2():
    GPIO.output(sel1,GPIO.HIGH)
    GPIO.output(sel2,GPIO.LOW)
    GPIO.output(sel3,GPIO.LOW)
def cam3():
    GPIO.output(sel1,GPIO.LOW)
    GPIO.output(sel2,GPIO.HIGH)
    GPIO.output(sel3,GPIO.LOW)
def cam4():
    GPIO.output(sel1,GPIO.HIGH)
    GPIO.output(sel2,GPIO.HIGH)
    GPIO.output(sel3,GPIO.LOW)
def cam5():
    GPIO.output(sel1,GPIO.LOW)
    GPIO.output(sel2,GPIO.LOW)
    GPIO.output(sel3,GPIO.HIGH)
def cam6():
    GPIO.output(sel1,GPIO.HIGH)
    GPIO.output(sel2,GPIO.LOW)
    GPIO.output(sel3,GPIO.HIGH)
def cam7():
    GPIO.output(sel1,GPIO.LOW)
    GPIO.output(sel2,GPIO.HIGH)
    GPIO.output(sel3,GPIO.HIGH)
def cam8():
    GPIO.output(sel1,GPIO.HIGH)
    GPIO.output(sel2,GPIO.HIGH)
    GPIO.output(sel3,GPIO.HIGH)

while(1):
    cam1()
    time.sleep(3)
    cam2()
    time.sleep(3)
    cam3()
    time.sleep(3)
    cam4()
    time.sleep(3)
    cam5()
    time.sleep(3)
    cam6()
    time.sleep(3)
    cam7()
    time.sleep(3)
    cam8()
    time.sleep(3)

GPIO.cleanup()
    
