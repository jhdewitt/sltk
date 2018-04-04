#!/usr/bin/python
# sl_control.py
# display structured light patterns via OSC
# capture images via mjpeg-streamer

# requires osc_display_client
# requires mjpeg-streamer (patched for valid timestamps)

import signal, os
import cv2
import sys
import urllib
import optparse
import numpy as np
import time
from Queue import Queue
from threading import Thread
from datetime import datetime, timedelta
from OSC import OSCServer, OSCClient, OSCMessage, OSCStreamingClient, OSCClientError
from subprocess import call
from subprocess import Popen
from sys import platform as _platform
from time import sleep


from slpy import *

final_destination = '~/completed_scans'
#accumulate_frame_n = 3

#################################################
## NETWORK
# 
# this program
control_server_addr = "0.0.0.0"
control_server_port = 4010

# mjpg-streamer (camera)
cam_server_addr = "0.0.0.0"
cam_server_port = "8080"

# sldisp (projector/display)
display_server_addr = "0.0.0.0"
display_server_port = 4021

# slcontrol GUI
GUI_server_addr = "0.0.0.0" # often an external computer
GUI_server_port = 4011

# slturn (turntable)
turn_server_addr = "0.0.0.0"
turn_server_port = 4040
#################################################


def quit_callback(path, tags, args, source):
    global run
    run = False

def cleanup():
    global turntable_enabled
    global streamer
    global run
    run = False
    #streamer.stop()
    server.close()
    if turntable_enabled:
        client_turntable_tcp.close()

def sighandler(signum, frame):
    print 'Signal handler called with signal', signum
    cleanup()

signal.signal(signal.SIGINT, sighandler)
signal.signal(signal.SIGTERM,sighandler)
signal.signal(signal.SIGHUP, sighandler)
signal.signal(signal.SIGQUIT,sighandler)


# always generate this sequence unless requested otherwise via OSC message
# graycode sequence 10 bit
# white, grays, red, green, blue
def default_sequence_buffer_plus_rgb():
    v = 0.4

    # seq
    gcbseq = GrayCodeBinaryPatternSequence(10,True,True,True,v,v,v)
    
    # seq
    rgbseq = MonochromePatternSequence(False,1.0,1.0,1.0)

    v0 = 1.0000
    v1 = 0.7530
    v2 = 0.5020
    v3 = 0.3608    
    v4 = 0.2510
    v5 = 0.1255
    
    rgbseq.addColor(v1,v1,v1)
    rgbseq.addColor(v2,v2,v2)
    rgbseq.addColor(v3,v3,v3)
    rgbseq.addColor(v4,v4,v4)
    rgbseq.addColor(v5,v5,v5)
    
    rgbseq.addColor(v2,0,0)
    rgbseq.addColor(0,v2,0)
    rgbseq.addColor(0,0,v2)

    # seqbuf
    newseqbuf = SequenceBuffer("geometrycolor")
    newseqbuf.add_sequence(gcbseq)
    newseqbuf.add_sequence(rgbseq)

    return newseqbuf


def default_sequence_buffer_rgb():
    v = 0.4

    # seq
    rgbseq = MonochromePatternSequence(False,1.0,1.0,1.0)

    v0 = 1.0000
    v1 = 0.7530
    v2 = 0.5020
    v3 = 0.3608    
    v4 = 0.2510
    v5 = 0.1255
    
    rgbseq.addColor(v1,v1,v1)
    rgbseq.addColor(v2,v2,v2)
    rgbseq.addColor(v3,v3,v3)
    rgbseq.addColor(v4,v4,v4)
    rgbseq.addColor(v5,v5,v5)
    
    rgbseq.addColor(v2,0,0)
    rgbseq.addColor(0,v2,0)
    rgbseq.addColor(0,0,v2)

    # seqbuf
    newseqbuf = SequenceBuffer("color")
    newseqbuf.add_sequence(rgbseq)

    return newseqbuf


def default_sequence():
    v = 0.98
    gcbseq = GrayCodeBinaryPatternSequence(10,True,True,True,v,v,v)

    newseqbuf = SequenceBuffer("geometry")
    newseqbuf.add_sequence(gcbseq)
    
    return newseqbuf

#def default_sequence():
#    v = 0.98
#    return GrayCodeBinaryPatternSequence(10,True,True,True,v,v,v)

# this method of reporting timeouts only works by convention
# that before calling handle_request() field .timed_out is 
# set to False
def handle_timeout(self):
    self.timed_out = True

# OSC service requests to activate turntable
def remote_turn_callback(path, tags, args, source):
    turn_dir   = min(max(int(args[0]),0),1)
    turn_steps = max(int(args[1]),1)
    print("RECEIVED REMOTE TURN EVENT (%d,%d)\n" % (turn_dir, turn_steps))
    send_turntable_command(turn_dir,turn_steps)

# "internal" trigger to begin sequence
def start_capture(name):
    global seqbuf
    
    print("SCAN NAME = %s\n" % name)
    if seqbuf.go == False:
        now = time.time()
        get_new_folder(name)
        print("%0.6f : SEQUENCE START" % now)
        print("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
        print("+ Sequence \"%s\" #%d -> [%s]" % (name,cap_number,cap_destination))
        print("+    [ %d ] scans of [ %d ] patterns per, with [ %d ] steps rotation between" % (seqbuf.runs,seqbuf.num_pats(),seqbuf.steps))
        print("+    consisting of:")
        sys.stdout.write(seqbuf.log())
        sys.stdout.write("\n")

        # kickoff buffer of sequences
        seqbuf.start_t = now
        seqbuf.go = True

        # kickoff first sequence
        seqbuf.seq[0].start_t = now
        seqbuf.seq[0].go = True
        
        seqbuf.name = name

        
        if _platform == "darwin":
            Popen(['say', 'scan %s'%cap_number])

# OSC trigger to begin sequence
def start_capture_callback(path, tags, args, source):
    global seqbuf

    # parameters for sequence:
    # (number of scans, number of steps to turn between each, name of sequence)
    num_scans = max(int(args[0]),1)
    num_steps = max(int(args[1]),0)
    name      =     str(args[2])
    
    # please cancel sequence before starting
    if seqbuf.go == False:
        
        seqbuf.reset()
        seqbuf = default_sequence_buffer_plus_rgb()
        #seqbuf = default_sequence_buffer_rgb()

        seqbuf.runs       = num_scans
        seqbuf.steps      = num_steps
        seqbuf.runs_total = num_scans
        
        start_capture(name)


       # start_capture(name)        

# OSC trigger to stop sequence
def cancel_capture_callback(path, tags, args, source):
    global seqbuf

    # please start sequence before cancel
    if seqbuf.go == True :
        seqbuf.stop_t = time.time()
        if _platform == "darwin":
            Popen(["say", "sequence aborted on frame %s"%seqbuf.seq[seqbuf.idx].idx])
        for subseq in seqbuf.seq:
            subseq.clear_list()
        seqbuf.idx = 0
        seqbuf.go = False
        seqbuf.completed = True
        remote_progress_update(0,0)
        print("STOP @ ", seqbuf.stop_t)
    send_monochrome_pattern(MonochromePattern(False,0.1,0.1,0.1))


def manual_pattern_callback(path, tags, args, source):
    global max_bit
    global use_bit
    global use_axis
    global use_inverted
    global use_r
    global use_g
    global use_b

    hit = False
    if path.find('/manual/bit') != -1 and args[0] != 0:
        hit = True
        print("MANUAL BIT: ", args[0])
        use_bit += args[0]
        use_bit = min(max(use_bit,0),max_bit)
        pat = GrayCodeBinaryPattern(use_axis, use_bit, use_inverted)
        pat.setcolor(use_r,use_g,use_b)
        send_graycode_pattern(pat)
        if _platform == "darwin":
            Popen(["say", "bit %d"%use_bit])
    
    if path.find('/manual/axis') != -1 and args[0] > -1:
        hit = True
        print("MANUAL AXIS: ", args[0])
        use_axis = args[0]
        use_axis = min(max(use_axis,0),1)
        pat = GrayCodeBinaryPattern(use_axis, use_bit, use_inverted)
        pat.setcolor(use_r,use_g,use_b)
        send_graycode_pattern(pat)
        if _platform == "darwin":
            Popen(["say", "%s axis"%('x' if use_axis==0 else 'y')])
    
    if path.find('/manual/inv') != -1 and args[0] > -1:
        hit = True
        print("MANUAL AXIS: ", args[0])
        use_inverted = args[0]
        use_inverted = min(max(use_inverted,0),1)
        pat = GrayCodeBinaryPattern(use_axis, use_bit, use_inverted)
        pat.setcolor(use_r,use_g,use_b)
        send_graycode_pattern(pat)
        if _platform == "darwin":
            Popen(["say", "%s"%('normal' if use_inverted==0 else 'invert')])

    if hit:
        print(use_axis,use_bit,use_inverted,use_r,use_g,use_b)



def osc_update():
    global server
    # do OSC server stuff
    server.timed_out = False
    while not server.timed_out:
        server.handle_request()

def main():
    global turntable_enabled
    global streamer
    global stream
    global run
    global seqbuf    
    global server
    global ts
    global bytes
    global destination
    global cap_number


    global client_display
    global control_server

    op = optparse.OptionParser(description="slcapture.py structured light capture program - talks to mjpeg-streamer and sldisp+slturn")
    op.add_option("-t", "--turntable", action="store_true", dest="turntable", help="Enable turntable OSC connection")
    op.set_defaults(turntable=False)
    (opts, args) = op.parse_args()

    # OSC control server (this program)
    server = OSCServer( (control_server_addr, control_server_port) )
    server.timeout = 0
    
    # funny python's way to add a method to an instance of a class
    import types
    server.handle_timeout = types.MethodType(handle_timeout, server)

    # OSC display client (connected to projector/monitor)
    client_display = OSCClient()
    client_display.connect( (display_server_addr, display_server_port) )

    # OSC control sketch (GUI)
    control_server = OSCClient()
    control_server.connect( (GUI_server_addr, GUI_server_port) )
    
    # OSC turntable client (connected to USB HID turntable)
    client_turntable_tcp = OSCStreamingClient()
    #client_turntable = OSCClient()
    
    if opts.turntable:
        print("TURNTABLE ENABLED!\n")
        client_turntable_tcp.connect( (turn_server_addr, turn_server_port) )
        #client_turntable.connect( (turn_server_addr, turn_server_port) )
        turntable_enabled = True
    else:
        turntable_enabled = False

    # data path
    destination = final_destination

    # MJPEG streamer client (connected to camera)
    stream=urllib.urlopen('http://%s:%s/?action=stream' % (cam_server_addr,cam_server_port))
    bytes=''

    ts = TimeState(time.time())

    #image_N = 0
    #image_T = -1
    #image_T_last = -1
    #image_T_local = -1
    #image_T_local_last = -1
    #image_dT_local = 1.0
    #image_dT_avg = 1.0
    #image_dT_sm = 0.9
    #START_T = time.time()
    #late_T_avg = 0.1

    max_bit = 11
    use_axis= 0
    use_bit = 0 
    use_inverted = 0
    use_r = 1
    use_g = 1
    use_b = 1

    cap_destination = ''
    cap_number = 0

    # main sequence buffer
    seqbuf = default_sequence_buffer_plus_rgb()

    server.addMsgHandler( "/turn", remote_turn_callback )
    server.addMsgHandler( "/start_capture", start_capture_callback )
    server.addMsgHandler( "/cancel_capture", cancel_capture_callback )
    server.addMsgHandler( "/manual/axis", manual_pattern_callback )
    server.addMsgHandler( "/manual/bit", manual_pattern_callback )
    server.addMsgHandler( "/manual/inv", manual_pattern_callback )
    server.addMsgHandler( "/manual/r", manual_color_callback )
    server.addMsgHandler( "/manual/g", manual_color_callback )
    server.addMsgHandler( "/manual/b", manual_color_callback )
    server.addMsgHandler( "/latency_check", remote_latency_check_callback )

    server.addMsgHandler( "/pattern_state", pattern_state_callback )
    #server.addMsgHandler( "/quit", quit_callback )

    #sequence = []
    #sequence = default_sequence()

    itr_t = time.time()
    itr_t_last = time.time()

    #streamer = mjpgStreamerThread().start()
    
    run = True
    # loop forever until quit
    while run:
        sleep(1/240.0)
        osc_update()
        each_frame()

    cleanup()
    
# loop
def each_frame():
    global bytes
    global stream
    global ts
    global seqbuf    
    global cap_destination

    # read mjpeg stream data
    #bytes+=stream.read(81920/4)
    #bytes+=stream.read(int(max(ts.img_size_avg*0.5,12800)))

    desired_bytes = ts.img_size_avg
    if ts.img_size_last < desired_bytes:
        desired_bytes = ts.img_size_last
    #desired_bytes = int(max(desired_bytes*0.5,12800))
    #desired_bytes = 32000
    desired_bytes = int(min(max(desired_bytes*0.95,12800),12800*50))
    
    bytes+=stream.read(desired_bytes)
    #bytes+=stream.read(81920*4)
    t = bytes.find('Timestamp')
    t2 = t+bytes[t:t+40].find(".")
    a = bytes.find('\xff\xd8')
    b = bytes.find('\xff\xd9')
    
    jpg = ''
    frame_time = 0
    new_frame = False

    # found JPG
    if a!=-1 and b!=-1:
        new_frame = True

        # extract image timestamp
        tstamp_s = bytes[t+11:t2]
        tstamp_ss = bytes[t2+1:t2+7]
        frame_time = float(tstamp_s) + float('0.'+tstamp_ss)
        #print("timestamp_s,ss = (%s)(%s)(%s)\n" % (bytes[t:t+50],tstamp_s, tstamp_ss))
        
        ts.new_frame(time.time(),frame_time,abs(b-a))

        jpg   = bytes[a:b+2]
        bytes = bytes[b+2:]
        
        dt = ts.cur_ts-ts.last_ts
        tfps = 1/max(dt,0.01) if ts.frame_n>1 else 0.0
        
        sys.stdout.write("\r%0.6f : frame# %0.4d (%0.3f late) (%0.3f dt = %2dfps) (%8d B) (%8d B avg) (%5d B diff) show_dt = %0.3f; show_dtavg = %0.3f" % (ts.cur_framet, ts.frame_n, ts.ts_late, dt, int(tfps), ts.img_size_last, ts.img_size_avg, ts.img_size_last-ts.img_size_avg, ts.show_dt, ts.show_dtavg ))
        sys.stdout.flush()

    bufidx = seqbuf.idx
    seqidx = seqbuf.seq[bufidx].idx

    # check each new camera image, if a sequence is already in progress
    if new_frame and seqbuf.should_go():

        
        pat = seqbuf.seq[bufidx].pat[seqidx]
        
        # the immediate frame index is at least 3 greater than the index at the time the current pattern was shown
        cond_framedelay = (ts.frame_n > pat.show_idx + 3)
        # the immediate frame timestamp is at least 0.15 seconds after the current pattern was shown
        cond_timedelay  = (ts.cur_ts  > pat.show_t + max(ts.ts_late + 0.05,0.25))
        
        # wait until current pattern is live
        if pat.go and pat.shown and pat.show_idx > 0 and not pat.saved and cond_framedelay and cond_timedelay:
            # progress percentage
            seqP = (seqbuf.runs_total-seqbuf.runs) / float(max(seqbuf.runs_total-1,1))
            scanP = seqbuf.idx / float(max(len(seqbuf.seq)-1,1))
            remote_progress_update(scanP,seqP)

            # bytes to jpg
            i = cv2.imdecode(np.fromstring(jpg, dtype=np.uint8),cv2.IMREAD_COLOR)
            #frame_datetime = datetime.fromtimestamp(frame_time)
            
            if pat.kind == 'gcb':
                fname = '%0.6f_gcb_%s_img.jpg' % (ts.cur_ts, '%02d_%02d_%1d-%02d'%(pat.axis, pat.bit, pat.inv, seqidx ))
            if pat.kind == 'sin':
                fname = '%0.6f_sin_%s_img.jpg' % (ts.cur_ts, '%02d_%02f_%02f_%02d-%02d'%(pat.axis, pat.freq, pat.phase, pat.inv, seqidx ))
            if pat.kind == 'rgb':
                fname = '%0.6f_rgb_%s_img.jpg' % (ts.cur_ts, '%03d_%03d_%03d_%1d-%02d'%(int(pat.r*255), int(pat.g*255), int(pat.b*255), pat.inv, seqidx ))
             
            outpath = '%s/%s' % (cap_destination, fname)
            
            cv2.imwrite(outpath,i)
            
            delay = (ts.cur_ts-seqbuf.seq[bufidx].pat[seqidx].show_t)
            sys.stdout.write("\r%0.6f : frame# %0.4d saved [IDX %d] %0.6f secconds after display @[%s]\n" % (ts.cur_ts, ts.frame_n, seqidx, delay, outpath) )
            writemsg = "                         %d / %d patterns : %d / %d sequences written                         " % (seqidx+1,len(seqbuf.seq[bufidx].pat),bufidx,len(seqbuf.seq))
            print(writemsg)
            print("*" * len(writemsg))
            sys.stdout.write("\n")
            #cv2.imwrite('%s/%0.6f_[%s]_img.jpg' % (cap_destination, ts.cur_ts, frame_datetime.strftime("%Y-%m-%d_%H-%M-%S-%f")),i)
            
            # add filename to sequence
            seqbuf.seq[bufidx].commit_image(fname,ts.frame_n,ts.cur_framet)

            # TERMINATE seq
            # ran out of patterns in current sequence
            if seqbuf.seq[bufidx].idx >= len(seqbuf.seq[bufidx].pat):
                seqbuf.seq[bufidx].go = False
                seqbuf.seq[bufidx].completed = True
                seqbuf.seq[bufidx].stop_t = time.time()
                scanduration = (seqbuf.seq[bufidx].stop_t - seqbuf.seq[bufidx].start_t)
                
                # more sequence in buffer
                if seqbuf.idx < len(seqbuf.seq)-1:
                    seqbuf.idx += 1
                    seqbuf.seq[seqbuf.idx].go = True
                    seqbuf.seq[seqbuf.idx].start_t = time.time()

                # finished run
                else:
                    seqbuf.idx = 0
                    seqbuf.runs = max(seqbuf.runs-1,0)
                    seqbuf.write_list(cap_destination)

                    if _platform == "darwin":
                        Popen(["say", "scan %d completed in %d seconds"%(cap_number,int(scanduration))])
                    print("SEQUENCE DURATION: %0.3f seconds" % scanduration)
                    
                    # more scans to do
                    if seqbuf.runs > 0:
                        print("********* TURNTABLE EVENT *********\n")
                        send_turntable_command(1,sequence.steps)
                        sleep(8)
                        start_capture(sequence.name)
                    
                    # all done
                    else:
                        seqbuf.go = False
                        seqbuf.completed = True
                        seqbuf.stop_t = time.time()
                        sleep(3)
                        if _platform == "darwin":
                            Popen(["say", "sequence terminated"])
                        v = 0.0
                        send_monochrome_pattern(MonochromePattern(False,v,v,v))
                        # Popen(["slcrunch", "-b 1 -c"])
                        # process all the scan data we just saved
                        # shell script 
            else:
                #sequence.seq[sequence.idx].show_t = -1
                seqbuf.seq[bufidx].pat[seqidx].show_t = -1
        else:        
            # send pattern for display
            #if seqbuf.should_go():
            if seqbuf.idx < len(seqbuf.seq):
                if seqbuf.seq[seqbuf.idx].is_primed(3/60.0):
                    pat = seqbuf.seq[bufidx].pat[seqidx]
                    if not pat.go:
                        #print("SENDING NEW PATTERN !!!!!!\n")
                        tmpkind = pat.kind
                        if tmpkind == 'gcb':
                            send_graycode_pattern(pat)
                        if tmpkind == 'sin':
                            send_sinusoid_pattern(pat)
                        if tmpkind == 'rgb':
                            send_monochrome_pattern(pat)
                        pat.go = True
                    #if pat.go and not pat.shown and pat.go_t < time.time()-3:
                    #    send_graycode_pattern(pat)
    
    #print("(\'%0.6f idx=%d"%(time.time(),sequence.idx))
    #cv2.imshow('i',i)
    #if cv2.waitKey(1) == 27:
    #    exit(0)

#############################################
# sldisp control messages

# OSC GCB
def send_graycode_pattern(pat):
    #print("TEST------------------------__" + pat.kind)
    if pat.kind == 'gcb':
        pat.go_t = time.time()
        pat.go = True
        pat.shown = False
        tup = [int(0), int(pat.inv), int(pat.axis), int(pat.bit), float(pat.r), float(pat.g), float(pat.b)]
        sys.stdout.write("\n%0.6f : PATTERN REQUEST [%1d %1d %1d %2d %0.2f %0.2f %0.2f]\n"%(time.time(),tup[0],tup[1],tup[2],tup[3],tup[4],tup[5],tup[6]))
        client_display.send( OSCMessage("/pattern_state", tup ) )
        del tup

# OSC SIN
def send_sinusoid_pattern(pat):
    if pat.kind == 'sin':
        pat.go_t = time.time()
        pat.go = True
        tup = [int(1), int(pat.inv), int(pat.axis), float(pat.freq), float(pat.phase), float(pat.r), float(pat.g), float(pat.b)]
        sys.stdout.write("%0.6f : PATTERN REQUEST [%1d %1d %1d %2d %0.2f %0.2f %0.2f %0.2f]"%(time.time(),tup[0],tup[1],tup[2],tup[3],tup[4],tup[5],tup[6],tup[7]))
        #print("%0.6f"%time.time(),"PATTERN REQUEST",tup)
        client_display.send( OSCMessage("/pattern_state", tup ) )
        del tup

# OSC RGB
def send_monochrome_pattern(pat):
    if pat.kind == 'rgb':
        pat.go_t = time.time()
        pat.go = True
        tup = [int(2), int(pat.inv), float(pat.r), float(pat.g), float(pat.b)]
        sys.stdout.write("%0.6f : PATTERN REQUEST [%1d %1d %0.2f %0.2f %0.2f]"%(time.time(),tup[0],tup[1],tup[2],tup[3],tup[4]))
        #print("%0.6f"%time.time(),"PATTERN REQUEST",tup)
        client_display.send( OSCMessage("/pattern_state", tup ) )
        del tup

# OSC turn
def send_turntable_command(turn_dir,turn_steps):
    global turntable_enabled
    print("SENDING TURNTABLE COMMAND\n")
    turn_dir = 0 if turn_dir==0 else 1
    turn_steps = max(1,turn_steps)
    tup = [int(turn_dir), int(turn_steps)]
    if turntable_enabled:
        client_turntable_tcp.sendOSC( OSCMessage("/turn", tup ))
    else:
        print "(not really)\n"

# OSC outgoing progress bar to slcontrol
def remote_progress_update(scan_progress, sequence_progress):
    tup = [float(scan_progress), float(sequence_progress)]
    #print("%0.6f"%time.time(),"remote_progress_update",tup)
    try:
        control_server.send( OSCMessage("/progress", tup ) )
    except OSCClientError:
        print("error, could not connect to slcapture.py for progress update")
    del tup

# OSC WIP incoming request to estimate delay between pattern request and camera image changing
def remote_latency_check_callback(path, tags, args, source):
    global sequence
    sentinel_v   = min(max(int(args[0]),0),1)
    print("RECEIVED LATENCY CHECK EVENT (%d)\n" % (sentinel_v))
    
    sequence = default_sequence()
    sequence.idx = 0

    for i in range(1,4):
        send_monochrome_pattern(MonochromePattern(False,1.0,1.0,1.0))
        #sleep(2)
        #send_monochrome_pattern(MonochromePattern(False,0.0,0.0,0.0))
        #sleep(2)


# service requests for manual pattern changes


# OSC service requests for manual pattern changes
def manual_color_callback(path, tags, args, source):
    global max_bit
    global use_bit
    global use_axis
    global use_inverted
    global use_r
    global use_g
    global use_b
    if path.find('r') > 0:
        use_r = float(args[0])
    if path.find('g') > 0:
        use_g = float(args[0])
    if path.find('b') > 0:
        use_b = float(args[0])
    print("MANUAL COLOR: ", path, args[0])
    #send_graycode_pattern(use_axis,use_bit,use_inverted,use_r,use_g,use_b)
    print(use_axis,use_bit,use_inverted,use_r,use_g,use_b)

# OSC update from sldisp of the current display state
def pattern_state_callback(path, tags, args, source):
    global ts
    global seqbuf

    #print("PATTERN STATE CALLBACK",path,args)
    
    if seqbuf.should_go():
        patid = int(args[0])
        match = False
        
        seqidx = seqbuf.idx
        curseq = seqbuf.seq[seqidx]

        patidx = curseq.idx
        curpat = curseq.pat[patidx]

        the_inv=0
        the_axis=0
        the_bit = 0
        the_freq = 0
        the_phase = 0
        the_r = 0
        the_g = 0
        the_b = 0

        # GCB
        if curseq.go and patid == 0:
            the_inv  = int(args[1])
            the_axis = int(args[2])
            the_bit  = int(args[3])
            the_r  = float(args[4])
            the_g  = float(args[5])
            the_b  = float(args[6])
            
            if curpat.shown == False and curpat.go == True:
                if the_axis == curpat.axis and the_bit == curpat.bit and the_inv == curpat.inv\
                        and abs(the_r-curpat.r)<1e-2 and abs(the_g-curpat.g)<1e-2 and abs(the_b-curpat.b)<1e-2:
                    match = True
                    #print("%0.6f SHOWN"%time.time())

        # SIN
        if curseq.go and patid == 1:
            the_inv    = int(args[1])
            the_axis   = int(args[2])
            the_freq   = int(args[3])
            the_phase  = int(args[4])
            the_r    = float(args[5])
            the_g    = float(args[6])
            the_b    = float(args[7])

            if curpat.shown == False and curpat.go == True:
                if the_axis == curpat.axis and the_inv == curpat.inv\
                        and abs(the_r-curpat.r)<1e-2 and abs(the_g-curpat.g)<1e-2 and abs(the_b-curpat.b)<1e-2:
                    match = True

        # RGB
        if curseq.go and patid == 2:
            the_inv  = int(args[1])
            the_r  = float(args[2])
            the_g  = float(args[3])
            the_b  = float(args[4])
            
            if curpat.shown == False and curpat.go == True:
                if the_inv == curpat.inv and abs(the_r-curpat.r)<1e-2 and abs(the_g-curpat.g)<1e-2 and abs(the_b-curpat.b)<1e-2:
                    match = True

        # mark pattern as shown
        if match:
            curpat.show_t = time.time()
            curpat.show_idx = ts.frame_n
            curpat.shown = True
            
            dt = curpat.show_t - curpat.go_t
            curpat.dt = dt
            
            sys.stdout.write("\n%0.6f : MATCH "%curpat.show_t)
            if patid == 0:
                print(" gcb pattern  [%d, %d, %d, %0.1f, %0.1f, %0.1f] delay = %0.6f" % (the_inv,the_axis,the_bit,the_r,the_g,the_b, dt))
            if patid == 1:
                print(" sin pattern  [%d, %d, %f, %0.1f, %0.1f, %0.1f] delay = %0.6f" % (the_inv,the_axis,the_freq,the_r,the_g,the_b, dt))
            if patid == 2:
                print(" rgb pattern  [%d, %0.1f, %0.1f, %0.1f] delay = %0.6f" % (the_inv,the_r,the_g,the_b, dt))
            
            ts.show_dt = dt
            if ts.show_dt < ts.show_dtavg:
                ts.show_dtsm = 0.95
            else:
                ts.show_dtsm = 0.0
            ts.show_dtavg = ts.show_dtavg*ts.show_dtsm + ts.show_dt*(1-ts.show_dtsm)


# generate path to new directory for writing scan data
def get_new_folder(name):
    global destination
    global cap_number
    global cap_destination
    
    seq_number = 0
    if name=='':
        while True:
            tmp_dest = '%s/sequence_%04d' % (destination, seq_number)
            if os.path.exists(tmp_dest):
                seq_number += 1
            else:
                break
    else:
        tmp_dest = "%s/%s" % (destination, name)
    
    while True:
        cap_destination = '%s/scan_%04d' % (tmp_dest, cap_number)
        if os.path.exists(cap_destination):
            cap_number += 1
        else:
            break
    if not os.path.exists(cap_destination):
        os.makedirs(cap_destination)




# WIP separate class to encapsulate image consumer thread (connects to mjpg-streamer)
class mjpgStreamerThread:
    global cam_server_addr
    def __init__(self,src=0):
        self.new_frame = False
        self.last_timestamp = -1
        self.stream = urllib.urlopen('http://%s:%s/?action=stream' % (cam_server_addr,cam_server_port))
        self.bytes = ''
        self.jpg = ''
        self.count_frames = 0
        self.count_tzero = time.time()
        stream.read(81920)
        self.stopped = False

        self.avgsize = 8192 #125000

    def start(self):
        Thread(target=self.update, args=()).start()
        return self

    def update(self):
        while True:
            if self.stopped:
                return

            # read mjpeg stream data
            #self.bytes+=self.stream.read(81920)
            self.bytes+=self.stream.read(min(max(int(round(self.avgsize/1.9)),8192),125000))
            t = self.bytes.find('Timestamp')
            t2 = t+self.bytes[t:t+40].find(".")
            a = self.bytes.find('\xff\xd8')
            b = self.bytes.find('\xff\xd9')
            

            # parse a frame from stream and get timestamp
            jpg = ''
            frame_time = 0
            self.new_frame = False
            if a>0 and b>0 and b>a and t>0 and t2>0 and a>t2 and b>t2:
                
                sz = b-a
                instdiff = abs(self.avgsize-sz)
                if instdiff > 500:
                    s = 0.2
                elif instdiff > 250:
                    s = 0.9
                elif instdiff > 150:
                    s = 0.95
                else:
                    s = 0.999
                self.avgsize = (self.avgsize*s + (1-s)*(b-a))
                
                self.new_frame = True
            
                # extract image timestamp
                tstamp_s = self.bytes[t+11:t2]
                tstamp_ss = self.bytes[t2+1:t2+7]
                frame_time = float(tstamp_s) + float('0.'+tstamp_ss)
                #print("timestamp_s,ss = (%s)(%s)(%s)\n" % (bytes[t:t+50],tstamp_s, tstamp_ss))
                self.last_timestamp = frame_time
                #ts.new_frame(time.time(),frame_time)

                self.jpg = self.bytes[a:b+2]
                self.bytes= self.bytes[b+2:]

                img = cv2.imdecode(np.fromstring(self.jpg, dtype=np.uint8),cv2.IMREAD_GRAYSCALE)
                mean,stddev = cv2.meanStdDev(img)
                
                #outpath = '%s/%s' % (cap_destination, fname)
                #print(" writing : [%s]\n"%outpath)
                #cv2.imwrite(outpath,i)
                
                #print("[%0.6f] : %d %d ; %d %d [%s.%s][%0.6f][%0.6f] avg %d\n" %(time.time(),t,t2,a,b, tstamp_s, tstamp_ss,frame_time,time.time()-frame_time, self.avgsize));
                #print("[%0.6f] : %d %d ; %d %d [%0.6f late]; mean, stddev = [%0.3f, %0.3f] avg = %d b, %0.1f fps" %(time.time(),t,t2,a,b, time.time()-frame_time,mean[0],stddev[0], self.avgsize,1/(time.time()-frame_time)));
                

    def read(self):
        return self.jpg

    def stop(self):
        self.stopped = True

        

if __name__ == "__main__":
    main()
