#!/usr/bin/env python

import move_vocab
import rospy
from std_msgs.msg import Bool, String, Float64
import time

class Choreography(object):

    def __init__(self):
        rospy.init_node('choreography')

        self.vocab = move_vocab.MovementVocab()

        rospy.Subscriber('song_info', Float64, self.getMovesReady)
        rospy.Subscriber('beat_info', Float64, self.move)
        rospy.Subscriber('songplay', Bool, self.start)
        self.readypub = rospy.Publisher('moves_ready', Bool, queue_size=10)

        self.moves = []

        self.go_msg = Bool()
        self.go_msg.data = True


    def getMovesReady(self, msg):

        avg_beat_len = msg.data

        self.vocab.generateDMP(['tucked_right_arm', 'up'], 'right_arm', 'punch', num_bases=2)
        self.vocab.generateDMP(['up', 'tucked_right_arm'], 'right_arm', 'retract', num_bases=2)
        self.vocab.makeMoveFromDMP('right_arm', 'punch', 'up_punch', 4*avg_beat_len, 'up')
        self.vocab.makeMoveFromDMP('right_arm', 'retract', 'up_retract', 4*avg_beat_len, 'tucked_right_arm')
        
        self.moves = ['up_punch', 'up_retract']
      
        self.readypub.publish(self.go_msg)

        print('moves ready')

    def start(self, msg):
        ## start timer
        self.start = time.time()

    def move(self, msg):

        nextmove = self.moves.pop(0)
        self.moves.append(nextmove)
        update = time.time()

        curr_time_in_secs = update - self.start

        next_beat = msg.data

        if next_beat < curr_time_in_secs:
            self.start -= next_beat - curr_time_in_secs #recalibrate timer to sync

        rospy.sleep(next_beat - curr_time_in_secs)

        self.readypub.publish(self.go_msg)

        self.vocab.executeMove(nextmove)



if __name__ == "__main__":
    choreo = Choreography()

    rospy.spin()