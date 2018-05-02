#!/usr/bin/python


# Copyright (c) 2011 Cereproc Ltd.
#
# Permission is hereby granted, free of charge, to any person obtaining
# a copy of this software and associated documentation files (the
# "Software"), to deal in the Software without restriction, including
# without limitation the rights to use, copy, modify, merge, publish,
# distribute, sublicense, and/or sell copies of the Software, and to
# permit persons to whom the Software is furnished to do so, subject to
# the following conditions:
#
# The above copyright notice and this permission notice shall be
# included in all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
# EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
# NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
# LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
# OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
# WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
#

#
# This is a simple program to produce audio output using the CereVoice
# Engine API.
#

#
# Standard imports
#
import os
import sys
from roboy_communication_cognition.srv import Talk
from std_msgs.msg import Int32
from roboy_communication_cognition.msg import SpeechSynthesis

import rospy
import time
import rospkg

# Add CereVoice Engine to the path

sdkdir = rospy.get_param('/speech_synthesis/sdkPath')
engdir = os.path.join(sdkdir, 'cerevoice_eng', 'pylib')
auddir = os.path.join(sdkdir, 'cerevoice_aud', 'pylib')
sys.path.append(engdir)
sys.path.append(auddir)

print sdkdir
print engdir

#
# Cereproc imports
#
import cerevoice_eng
import cerevoice_aud


# User data class to store information we would like to have
# access to in the callback.
class EngineUserData:
    def __init__(self, wavout, engine, channel, player):
        # If false, audio is being played
        self.wavout = wavout
        # Audio player
        self.player = player
        # These two are optional, they can be used to control
        # cancellation in the callback function.
        self.engine = engine
        self.channel = channel

# Channel event callback function
class CereVoiceEngineCallback:
    def __init__(self):
        self.engine = cerevoice_eng.CPRCEN_engine_new()

        # Set the loading mode - all data to RAM or with audio and indexes ozn disk
        loadmode = cerevoice_eng.CPRC_VOICE_LOAD

        voicePath = rospy.get_param('/speech_synthesis/voicePath')
        licensePath = rospy.get_param('/speech_synthesis/licensePath')
        # Load the voice
        ret = cerevoice_eng.CPRCEN_engine_load_voice(self.engine, licensePath, "", voicePath, cerevoice_eng.CPRC_VOICE_LOAD_EMB)

        if not ret:
            sys.stderr.write("ERROR: could not load the voice, check license integrity\n")
            sys.exit(1)

        info = cerevoice_eng.CPRCEN_engine_get_voice_info(self.engine, 0, "VOICE_NAME")
        sample_rate = cerevoice_eng.CPRCEN_engine_get_voice_info(self.engine, 0, "SAMPLE_RATE")
        sys.stderr.write("INFO: voice name is '%s', sample rate '%s'\n" % (info, sample_rate))

        self.channel = cerevoice_eng.CPRCEN_engine_open_default_channel(self.engine)
        freq = int(cerevoice_eng.CPRCEN_channel_get_voice_info(self.engine, self.channel, "SAMPLE_RATE"))

        self.wavout = False
        self.player = cerevoice_aud.CPRC_sc_player_new(freq)

        self.userdata = EngineUserData(self.wavout, self.engine, self.channel, self.player)

        res = cerevoice_eng.engine_set_callback(self.engine, self.channel, self)

        # User-configurable parameter, could be as simple as a file
        # name for the output, or a richer data structure.
        #self.ws= serverThread(1, "Thread-1", 9090)
        #self.ws.start()
        self.pub = rospy.Publisher('/roboy/cognition/speech/synthesis', SpeechSynthesis, queue_size=10)
        self.t_pub = rospy.Publisher('/roboy/cognition/speech/synthesis/duration', Int32, queue_size=10)

    def cleanup(self):

        #t_0 = int(round(time.time() * 1000)) - t_0
        #print t_0

        play=True
        if play:
            while cerevoice_aud.CPRC_sc_audio_busy(self.player):
                cerevoice_aud.CPRC_sc_sleep_msecs(50)

        # Clean up
        cerevoice_aud.CPRC_sc_player_delete(self.player)
        cerevoice_eng.CPRCEN_engine_delete(self.engine)



    # The callback function must be called 'channel_callback'
    def channel_callback(self, data):
        # Get the audio buffer for this piece of synthesis output
        abuf = cerevoice_eng.data_to_abuf(data)
        # Print transcription information from the audio buffer
        # This information could be used for lip syncing, markers
        # can be used to send data to this application via the
        # engine.
        visemes=[]
        durations=[]
        speech_duration = 0
        for i in range(cerevoice_eng.CPRC_abuf_trans_sz(abuf)):
            trans = cerevoice_eng.CPRC_abuf_get_trans(abuf, i)
            if trans:
                start = cerevoice_eng.CPRC_abuf_trans_start(trans)
                end = cerevoice_eng.CPRC_abuf_trans_end(trans)
                name = cerevoice_eng.CPRC_abuf_trans_name(trans)
                if cerevoice_eng.CPRC_abuf_trans_type(trans) == cerevoice_eng.CPRC_ABUF_TRANS_PHONE:
                    #print "INFO: phoneme '%s', start '%s', end '%s'" % (name, start, end)
                    #print "INFO: viseme '%d'" % getViseme(name)
                    visemes.append(name)
                    durations.append(end-start)
                    speech_duration += (end-start)
                '''elif cerevoice_eng.CPRC_abuf_trans_type(trans) == cerevoice_eng.CPRC_ABUF_TRANS_WORD:
                    print "INFO: word '%s', start '%s', end '%s'" % (name, start, end)
                elif cerevoice_eng.CPRC_abuf_trans_type(trans) == cerevoice_eng.CPRC_ABUF_TRANS_MARK:
                    print "INFO: marker '%s', start '%s', end '%s'" % (name, start, end)
                else:
                    print "WARNING: transcription type '%s' not known" % cerevoice_eng.CPRC_abuf_trans_type(trans)'''
        # Save the output.  Creates a new file on the first call,
        # appends on subsequent calls
        if self.userdata.wavout:
            # print "INFO: appending to ", self.userdata.wavout
            cerevoice_eng.CPRC_riff_append(abuf, self.userdata.wavout)
        else:
            # Cue the audio for playing
            if self.userdata.player:
                buf = cerevoice_aud.CPRC_sc_audio_short_disposable(cerevoice_eng.CPRC_abuf_wav_data(abuf),
                                                                   cerevoice_eng.CPRC_abuf_wav_sz(abuf))
                cerevoice_aud.CPRC_sc_audio_cue(self.userdata.player, buf)

        speech_duration *= 1000
        #print "speech duration: ", speech_duration

        for v in range (0, len(visemes)):
            if v == (len(visemes)-2):
                self.t_pub.publish(speech_duration)
            #message = "0 " + str(visemes[v]) + " " + str(durations[v]*1000.0)

            phonemes = SpeechSynthesis()
            phonemes.phoneme = visemes[v]
            phonemes.duration = durations[v]
            phonemes.header.stamp = rospy.Time(durations[v])
            self.pub.publish(phonemes)
            time.sleep(durations[v])

    def synthesize(self, message):
        cerevoice_eng.CPRCEN_engine_channel_speak(self.engine, self.channel, message, len(message), 1)


    def talk_callback(self,req):
        self.synthesize(req.text)
        return {'success':True}


if __name__ == '__main__':
    rospy.init_node('speech_synthesis_server')
    cerevoice_class = CereVoiceEngineCallback()
    rospy.on_shutdown(cerevoice_class.cleanup)
    cerevoice_class.synthesize("Speech synthesis is ready now")
    s_sys = rospy.Service('/roboy/cognition/speech/synthesis/talk', Talk, cerevoice_class.talk_callback)
    print "Ready to /roboy/cognition/speech/synthesis/talk."

    rospy.spin()
