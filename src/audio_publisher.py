#!/usr/bin/env python

import pyaudio
import wave
import rospy
import numpy as np
from audio_common_msgs.msg import AudioData
from std_msgs.msg import String, Bool, Float64
import btrack
# import audioowl
from scipy.io import wavfile


class AudioPublisher():
    def __init__(self, song="/home/mgessner/Downloads/Mariah Carey All I Want For Christmas Is You.wav", CHUNK = 1024):
        self.aud = pyaudio.PyAudio()
        self.playing = False
        self.chunk_size = CHUNK

        rospy.init_node("audio_publisher")



        self.rawpub = rospy.Publisher('raw_audio', AudioData, queue_size=20)
        self.beatpub = rospy.Publisher('beat_info', Float64, queue_size=10)
        self.songpub = rospy.Publisher('song_info', Float64, queue_size=10)
        self.open_stream(song)
        rospy.Subscriber('shutup', Bool, self.stop)
        rospy.Subscriber('songplay', Bool, self.play)
        return

    def __del__(self):
        self.aud.terminate()

    def open_stream(self, song):
        wf = wave.open(song, 'rb')
        self.stream = self.aud.open(format=self.aud.get_format_from_width(wf.getsampwidth()),
                channels=wf.getnchannels(),
                rate=wf.getframerate(),
                output=True, output_device_index=1)
        self.wf = wf
        sr, wholedata = wavfile.read(song)
        # self.beats = btrack.trackBeats(wholedata).tolist()
        onsetDF = btrack.calculateOnsetDF(wholedata)
        ODFbeats = btrack.trackBeatsFromOnsetDF(onsetDF)
        diffs = ODFbeats[1:] - ODFbeats[0:len(ODFbeats)-1]

        self.beats = ODFbeats.tolist()

        msg = Float64()
        msg.data = np.average(diffs)
        self.songpub.publish(msg)

        print('beats extracted!')
        print(self.beats)




    def play(self, msg):
        self.playing = True
        data = self.wf.readframes(self.chunk_size)

        chunklen = self.chunk_size / float(self.wf.getframerate())
        chunktotal = 0
        i = 0

        while self.playing and (data != ''):
            self.stream.write(data)
            data = self.wf.readframes(self.chunk_size)
            secs = chunktotal / self.wf.getframerate()
            disttobeat = self.beats[0] - secs
            if disttobeat < chunklen:
                msg = Float64()
                msg.data = self.beats.pop(0)
                self.beatpub.publish(msg)
                i += 1
            chunktotal += self.chunk_size

            msg = AudioData()
            msg.data = data
            self.rawpub.publish(msg)
            # self.publish(data, secs)

            # print("seconds:", chunktotal / float(self.wf.getframerate()))
        
        self.stop()


    def stop(self, msg=True):
        print("hello", msg)
        self.playing = False
        self.stream.stop_stream()
        self.stream.close()



if __name__ == "__main__":

    CHUNK = 1024
    # song = "/home/mgessner/Downloads/Mariah Carey All I Want For Christmas Is You.wav"
    song = "/home/mgessner/Downloads/Wham! - Last Christmas (Official Video).wav"


    ap = AudioPublisher(song, CHUNK)
    # ap.play()

    rospy.spin()

