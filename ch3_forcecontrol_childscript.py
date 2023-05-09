# -*- coding: utf-8 -*-
"""
Created on Mon May  8 20:35:08 2023

@author: rraghur
"""

#python
#do not run the code remotely but create a non-threaded python child script on the exoskeletonlifting_forcecontrol.ttt model and upload this code below
import math

def sysCall_init():
    # do some initialization here
    global j, timer, theta, weight, length, Inertia, alpha_d1, alpha_d2, torq_ext, K, D, vel_d, pos_d
    j           = sim.getObject("/Bill/legJoint")       #get handle of the leg joint that connects torso to legs
    timer       = sim.getSimulationTime()               #used to record current simulation time
    theta       = 0                                     #angle covered                                    
    weight      = 49.84                                 #weight of box+hands+forearms+upperarms+head+torso - refer chapter for calculations
    length      = 0.66                                  #length of torso
    Inertia     = 7.56                                  #mass moment of Inertia of upper body - refer chapter for calculations
    alpha_d1    = -4.187                                #Angular acceleration for the half of the lifting period, negative becuase of rotation direction
    alpha_d2    = 4.187                                 #Angular acceleration for the second-half of the lifting period               
    torq_ext    = -120                                  #External moment provided by the person using hip muscles
    K           = 500                                   #Stiffness constant
    D           = 300                                   #Damping coefficient
    vel_d       = 0                                     #Initial desired velocity
    pos_d       = 70*3.14/180                           #Initial desired position
    sim.setJointPosition(j,70*3.14/180)                 #Setting the starting lifting position at 70 degrees
    pass

def sysCall_actuation():
    # put your actuation code here
    global j, timer, theta, time, weight, length, Inertia, alpha_d1, alpha_d2, torq_ext, K, D, vel_d, pos_d
    
    while sim.getSimulationTime()-timer >= 0.017 and sim.getSimulationTime() <= 0.5:            #for the first half of lifting from 70 to 40 degrees
        pos         = sim.getJointPosition(j)
        vel         = sim.getJointVelocity(j)
        torq_d      = (Inertia*alpha_d1 - weight*0.5*length*math.cos(theta+20*3.14/180)*9.81)   #desired torque values for completing lifting action
        torq        = (K*(pos_d - pos) + D*(vel_d - vel)) + torq_d - torq_ext                   #impedance controller
        sim.setJointTargetForce(j,torq_ext+torq)
        theta       = theta + 1*3.14/180
        pos_d       = 70*3.14/180-theta
        torq_ext    = torq_ext + 2
        vel_d       = alpha_d1*timer
        timer       = sim.getSimulationTime()
        print(pos)                                                                              #reference position (in radians)

    while sim.getSimulationTime()-timer >= 0.0017 and sim.getSimulationTime() <= 1:             #for the second half of lifting from 40 to 10 degrees
        pos         = sim.getJointPosition(j)
        vel         = sim.getJointVelocity(j)
        torq_d      = (Inertia*alpha_d2 - weight*0.5*length*math.cos(theta+20*3.14/180)*9.81)
        torq        = (K*(pos_d - pos) + D*(vel_d - vel)) + torq_d - torq_ext                   #impedance controller
        sim.setJointTargetForce(j,torq_ext+torq)
        theta       = theta + 1*3.14/180
        pos_d       = 70*3.14/180-theta
        torq_ext    = torq_ext + 2
        vel_d       = -2.0935 + alpha_d2*(timer - 0.5)
        timer       = sim.getSimulationTime()
        print(pos)

    if sim.getSimulationTime()>=1:                                                              #stop simulation after 1 second or when the lifting is complete
        sim.stopSimulation()
        
    pass

def sysCall_sensing():
    # put your sensing code here
    pass

def sysCall_cleanup():
    # do some clean-up here
    pass

# See the user manual or the available code snippets for additional callback functions and details
