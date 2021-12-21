#!/usr/bin/env python

import rospy
import moveit_commander
from si_utils import jaco_jtas_test
from moveit_commander.interpreter import MoveGroupCommandInterpreter
import yaml
import os
import pickle

import roslib; 
roslib.load_manifest('dmp')
import numpy as np
from dmp.srv import *
from dmp.msg import *

class MovementVocab(object):
    def __init__(self, dims=6, dt = 0.1, K=100):
        self.interpreter = MoveGroupCommandInterpreter()
        moveit_commander.roscpp_initialize(sys.argv)
        # rospy.init_node('move_vocab')
        self.remembered_poses = {}
        self.groups = {}
        self.loadPoses()
        self.moves = {}
        self.dmps = {}
        self.dims = dims
        self.dt = dt
        self.K = K
        self.D = 2.0 * np.sqrt(K)

    def loadPoses(self, filename="move_group.cfg"):

        self.interpreter.execute_generic_command("load")
        for group_name in self.interpreter.get_loaded_groups():
            self.interpreter.execute_generic_command('use '+group_name)
            self.groups[group_name] = self.interpreter.get_active_group()
            self.remembered_poses[group_name] = self.groups[group_name].get_remembered_joint_values()

    def generateDMP(self, pose_list, group_name, dmp_name, num_bases=8):
        ''' Given pose list and name for trajectory implied by pose list, generate and save a DMP'''

        traj = []          
        if type(pose_list[0]) == str:
            for pose in pose_list:
                traj.append(self.remembered_poses[group_name][pose])
        else:
            traj = pose_list
        resp = makeLFDRequest(self.dims, traj, self.dt, self.K, self.D, num_bases)

        print(resp)
        self.dmps[dmp_name] = resp

    def makeMoveFromDMP(self, group_name, dmp_name, move_name, desired_time, goal):
        if self.dmps[dmp_name] is None:
            print("no plan available pick a different movement")
            return
    
        dmp = self.dmps[dmp_name]
        makeSetActiveRequest(dmp.dmp_list)

        group = self.groups[group_name]
        current = group.get_current_joint_values()
        print(current)

        #Now, generate a plan
        x_0 = current          #Plan starting at a different point than demo 
        x_dot_0 = [0.0]*6   
        t_0 = 0                
        goal_thresh = [0.1]*6
        seg_length = -1          #Plan until convergence to goal
        tau = desired_time       #Desired plan should take twice as long as demo
        dt = self.dt
        integrate_iter = 5       #dt is rather large, so this is > 1  
        if type(goal) == str:
            goal = self.remembered_poses[group_name][goal]
            if goal is None:
                print('goal not known, pick a different movement')
                return
        plan = makePlanRequest(x_0, x_dot_0, t_0, goal, goal_thresh, 
                            seg_length, tau, dt, integrate_iter)

        print(plan)

        self.moves[move_name] = plan
        return plan

    def executeMove(self, move_name):
        self.moving = True
        executePlan(self.moves[move_name])

    def saveMovements(self):
        with open('moves.pickle', 'wb') as f:
            pickle.dump(self.moves, f, pickle.HIGHEST_PROTOCOL)
    
    def saveDMPs(self):
        with open('dmps.pickle', 'wb') as f:
            pickle.dump(self.dmps, f, pickle.HIGHEST_PROTOCOL)

    def saveAll(self):
        self.saveMovements()
        self.saveDMPs()

    def loadDMPs(self):
        with open('dmps.pickle', 'wb') as f:
            self.dmps = pickle.load(f)

    def loadMovements(self):
        with open('moves.pickle', 'wb') as f:
            self.moves = pickle.load(f)
    
    def loadAll(self):
        self.loadDMPs()
        self.loadMovements()

def executePlan(dmpPlan):

    traj = jaco_jtas_test.JacoJTASTest()
    for i, point in enumerate(dmpPlan.plan.points):
        traj.add_point(point.positions, dmpPlan.plan.times[i], point.velocities)

    traj.start()

    traj.wait(dmpPlan.plan.times[-1])
    print("Exiting - Joint Trajectory Action Test Complete")

#Learn a DMP from demonstration data
def makeLFDRequest(dims, traj, dt, K_gain, 
                D_gain, num_bases):
    demotraj = DMPTraj()
        
    for i in range(len(traj)):
        pt = DMPPoint()
        pt.positions = traj[i]
        demotraj.points.append(pt)
        demotraj.times.append(dt*i)
            
    k_gains = [K_gain]*dims
    d_gains = [D_gain]*dims
        
    print("Starting LfD...")
    rospy.wait_for_service('learn_dmp_from_demo')
    try:
        lfd = rospy.ServiceProxy('learn_dmp_from_demo', LearnDMPFromDemo)
        resp = lfd(demotraj, k_gains, d_gains, num_bases)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
    print("LfD done")
            
    return resp

#Set a DMP as active for planning
def makeSetActiveRequest(dmp_list):
    try:
        sad = rospy.ServiceProxy('set_active_dmp', SetActiveDMP)
        sad(dmp_list)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

#Generate a plan from a DMP
def makePlanRequest(x_0, x_dot_0, t_0, goal, goal_thresh, 
                    seg_length, tau, dt, integrate_iter):
    print("Starting DMP planning...")
    rospy.wait_for_service('get_dmp_plan')
    try:
        gdp = rospy.ServiceProxy('get_dmp_plan', GetDMPPlan)
        resp = gdp(x_0, x_dot_0, t_0, goal, goal_thresh, 
                seg_length, tau, dt, integrate_iter)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
    print("DMP planning done")
            
    return resp

# if __name__ == '__main__':

#     maker = MovementVocab()
#     # choreo.loadMovements()

#     maker.generateDMP(['tucked_right_arm', 'elbow_out', 'tucked_right_arm'], 'right_arm', 'nudge', num_bases=2)
#     plan = maker.makeMoveFromDMP('right_arm', 'nudge', 'quick nudge', 2.0, maker.remembered_poses['right_arm']['tucked_right_arm'])
#     executePlan(plan)

#     maker.saveAll()

#     rospy.spin()





