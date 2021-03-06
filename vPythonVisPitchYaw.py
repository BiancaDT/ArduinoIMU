from vpython import *
from time import *
import numpy as np
import math
import serial

ad=serial.Serial('com4', 115200)

scene.range=5
toRad=2*np.pi/360
toDeg=1/toRad

scene.forward=vector(-1,-1,-1)
scene.width=(600)
scene.height=(600)

Xarrow=arrow(axis=vector(1,0,0), length=2, shaftwidth=.1,color=color.green)
Yarrow=arrow(axis=vector(0,1,0), length=2, shaftwidth=.1,color=color.blue)
Zarrow=arrow(axis=vector(0,0,1), length=2, shaftwidth=.1,color=color.red)


bBOard=box(length=1.2,width=.8,height=.1,pos=vector(-0.5,.1+.05,0), color=color.white)
imu9=box(length=.5,width=.5,height=.1,pos=vector(-0.5,.1+.05+.1,0), color=color.red)

arduin=box(length=3,width=1.8,height=.2,color=color.white, opacity=.5)
arduinColor=box(length=1,width=1.8,height=.2,color=color.cyan, opacity=.3, pos=vector(1,.1,0))

frontArrow=arrow(axis=vector(1,0,0), length=4, shaftwidth=.1,color=color.purple)
upArrow=arrow(axis=vector(0,1,0), length=1, shaftwidth=.1,color=color.magenta)
sideArrow=arrow(axis=vector(0,0,1), length=4, shaftwidth=.1,color=color.orange)

myObj=compound([bBOard,imu9,arduin,arduinColor])

while True:
    while ad.inWaiting()==0:
        pass
    dataPacket=ad.readline()
    dataPacket=str(dataPacket, 'utf-8')
    splitPacket=dataPacket.split(",")

    roll = float(splitPacket[0])*toRad
    pitch = float(splitPacket[1])*toRad
    yaw = float(splitPacket[2])*toRad
    print("Roll=",roll*toDeg, "Pitch=",pitch*toDeg, "Yaw=",yaw*toDeg)

    rate(40)
    k=vector(cos(yaw)*cos(pitch),sin(pitch),sin(yaw)*cos(pitch))

    y=vector(0,1,0)
    s=cross(k,y)
    v=cross(s,k)

    vrot=v*cos(-roll)+cross(k,v)*sin(-roll)

    frontArrow.axis=k
    sideArrow.axis=s
    upArrow.axis=v

    myObj.axis=k
    myObj.up=vrot

    sideArrow.length=2
    frontArrow.length=4
    upArrow.length = 1
