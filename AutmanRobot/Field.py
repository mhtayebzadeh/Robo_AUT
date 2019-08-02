#!/usr/bin/env python

class Field:
    fieldZones = []
    def __init__(self):
        self.fieldZones.append(Zone(0,0,0,"start_zone" , 0))
        self.fieldZones.append(Zone(1,0,2,"goal_zone"))

    def getFieldZones(self):
        return self.fieldZones
        
class Zone:
    id = 0
    name = ""
    x_pos = 0
    y_pos = 0
    ballInZone = -1 
    def __init__(self,id,x_pos,y_pos,name = "_" , ballInZone = -1):
        self.id = id
        self.x_pos = x_pos
        self.y_pos = y_pos
        self.name = name
        self.ballInZone = ballInZone
    
    def setBallInZone(self,n):
        self.ballInZone = n

