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
from datetime import datetime, timedelta
from OSC import OSCServer, OSCClient, OSCMessage, OSCStreamingClient
from time import sleep
from subprocess import call
from subprocess import Popen
from sys import platform as _platform

final_destination = "~/completed_scans"

cam_server_addr = "0.0.0.0"
cam_server_port = 4011

turn_server_addr = "0.0.0.0"
turn_server_port = 4013

GUI_server_addr = "0.0.0.0"
GUI_server_port = 12000

control_server_addr = "0.0.0.0"
control_server_port = 3210

# this method of reporting timeouts only works by convention
# that before calling handle_request() field .timed_out is 
# set to False
def handle_timeout(self):
    self.timed_out = True

class TimeState:
    start = -1
    frame_n = 0
    
    cur_framet = -1
    last_framet = -1

    cur_ts = -1
    last_ts = -1

    ts_late = -1
    ts_lateavg = -1
    ts_latesm = 0.9

    dt = 0.1
    dt_avg = 0.1
    dt_sm = 0.9
    
    show_dt = 0
    show_dtavg = 0
    show_dtsm = 0.9

    def __init__(self,now):
        start = now

    def new_frame(self,now,timestamp):
        self.frame_n = self.frame_n + 1
        self.last_framet = self.cur_framet
        self.cur_framet = now
        self.last_ts = self.cur_ts
        self.cur_ts = timestamp

        self.ts_late = self.cur_framet-self.cur_ts
        self.ts_lateavg = self.ts_lateavg*self.ts_latesm + self.ts_late*(1-self.ts_latesm)

        self.dt = self.cur_ts-self.last_ts
        self.dt_avg = self.dt_avg*self.dt_sm + self.dt*(1-self.dt_sm)

# pattern types

class StructuredLightPattern(object):
    kind = ''
    name = ''
    
    inv = 0
    r = 0
    g = 0
    b = 0

    go = False
    shown = False
    saved = False
    
    want_t = -1
    show_t = -1
    save_t = -1
    
    show_idx = -1

    def __init__(self):
        pass

    def setcolor(self,r,g,b):
        self.r = r
        self.g = g
        self.b = b

    def setname(name):
        self.name = name

class GrayCodeBinaryPattern(StructuredLightPattern):
    axis = 0
    bit = 0

    def __init__(self,inv,axis,bit):
        self.kind = 'gcb'
        self.axis = axis
        self.bit = bit
        self.inv = inv

class SinusoidPattern(StructuredLightPattern):
    axis = 0
    freq = 1
    phase = 0
    
    def __init__(self,inv,axis,freq,phase):
        self.kind = 'sin'
        self.inv = inv
        self.axis = axis
        self.freq = freq
        self.phase = phase

class MonochromePattern(StructuredLightPattern):
    def __init__(self,inv,r,g,b):
        self.kind = 'rgb'
        self.inv = inv
        self.r = r
        self.g = g
        self.b = b

# sequence types

class PatternSequence(object):
    kind = ''
    name = ''
    r = 1.0
    g = 1.0
    b = 1.0
    
    runs_total = 1
    runs = 1
    steps = 50
    
    go = False
    start_t = -1
    stop_t = -1
    
    idx = 0
    seq = []
    seq_path = []

    def __init__(self):
        pass

    def gen():
        pass

    def commit_image(self,path,newidx,newframetime):
        self.seq_path.append(path)
        self.seq[self.idx].saved = True
        self.seq[self.idx].save_t = time.time()
        self.seq[self.idx].show_t = newframetime
        #self.seq[self.idx].show_idx = newidx
        self.idx += 1
    
    def clear_list(self):
        del self.seq[:]
        del self.seq_path[:]
    
    def reset(self):
        self.go = False
        self.start_t = -1
        self.stop_t = -1
        self.idx = 0
        self.gen()
 
class GrayCodeBinaryPatternSequence(PatternSequence):
    kind = 'gcb_seq'
    
    bits = 0
    showx = True
    showy = False
    showinv = False
    frame_mult = 2 if (showx and showy) else 1
    frame_mult *= 2 if showinv else 1
    
    def __init__(self,bits,showx,showy,showinv,r,g,b):
        self.bits = bits
        self.showx = showx
        self.showy = showy
        self.showinv = showinv
        self.r = r
        self.g = g
        self.b = b
        self.gen()
        self.print_debug()
    
    def print_debug(self):
        for idx in range(len(self.seq)):
            pat = self.seq[idx]
            print('%3d %s %3d %3d %3d : (%0.3f %0.3f %0.3f)' %(idx,pat.kind,pat.axis,pat.bit,pat.inv,pat.r,pat.g,pat.b))

    def gen(self):
        self.clear_list()
        if self.showx:
            for x in range(self.bits):
                idx = self.bits-x-1
                pat = GrayCodeBinaryPattern(0,0,idx)
                pat.setcolor(self.r,self.g,self.b)
                #if idx == 4:
                #    pat.setcolor(1,0,0)
                self.seq.append(pat)
                if self.showinv:
                    patinv = GrayCodeBinaryPattern(1,0,idx)
                    patinv.setcolor(self.r,self.g,self.b)
                #    if idx == 4:
                #        patinv.setcolor(1,0,0)
                    self.seq.append(patinv)
        if self.showy:
            for y in range(self.bits):
                idx = self.bits-y-1
                pat = GrayCodeBinaryPattern(0,1,idx)
                pat.setcolor(self.r,self.g,self.b)
                self.seq.append(pat)
                if self.showinv:
                    patinv = GrayCodeBinaryPattern(1,1,idx)
                    patinv.setcolor(self.r,self.g,self.b)
                    #if y == 0:
                    #    patinv.setcolor(0.5,0.5,0.5)
                    self.seq.append(patinv)
    
    def write_list(self,outpath):
        f = open('%s/list.yaml'%outpath,'w')
        f.write("%YAML:1.0\ngcb_images:\n")
        for idx in range(len(self.seq_path)):
            f.write("   - \"%s\"\n" % self.seq_path[idx])
        if len(self.seq) != len(self.seq_path):
            f.write("gcb_images_N:%d\n"%len(self.seq));
            f.write("gcb_images_N_path:%d\n"%len(self.seq_path));
        f.close()
     
    def setname(name):
        self.name = name

    @classmethod
    def fromcolor(self,bits,showx,showy,showinv,r,g,b):
        self.bits = bits
        self.showx = showx
        self.showy = showy
        self.showinv = showinv
        self.r = r
        self.g = g
        self.b = b

class SinusoidPatternSequence(PatternSequence):
    kind = 'sin_seq'
    
    frames = 3 
    showx = True
    showy = False
    showinv = False
    frame_mult = 2 if (showx and showy) else 1
    frame_mult *= 2 if showinv else 1

    def __init__(self,frames,showx,showy,showinv,r,g,b):
        self.frames = frames
        self.showx = showx
        self.showy = showy
        self.showinv = showinv
        self.r = r
        self.g = g
        self.b = b
        self.gen()
        self.print_debug()

    def print_debug(self):
        for idx in range(len(self.seq)):
            pat = self.seq[idx]
            print('%3d %s %3d %3d %3d : (%0.3f %0.3f %0.3f)' %(idx,pat.kind,pat.axis,pat.freq,pat.phase,pat.r,pat.g,pat.b))

    def gen(self):
        self.clear_list()
        pat = 0
        if self.showinv:
            pat = SinusoidPattern(1-self.r,1-self.g,1-self.b)
        else:
            pat = SinusoidPattern(self.r,self.g,self.b)
        self.seq.append(pat);

    def write_list(self,outpath):
        f = open('%s/rgb_list.yaml'%outpath,'w')
        f.write("%YAML:1.0\nsin_images:\n")
        for idx in range(len(self.seq_path)):
            f.write("   - \"%s\"\n" % self.seq_path[idx])
        if len(self.seq) != len(self.seq_path):
            f.write("sin_images_N:%d\n"%len(self.seq));
            f.write("sin_images_N_path:%d\n"%len(self.seq_path));
        f.close()


class MonochromePatternSequence(PatternSequence):
    kind = 'rgb_seq'

    def addColor(self,r,g,b):
        pat = MonochromePattern(False,r,g,b)
        self.seq.append(pat)

    def __init__(self,inv,r,g,b):
        self.showinv = inv
        self.r = r
        self.g = g
        self.b = b
        self.gen()
        self.print_debug()

    def print_debug(self):
        for idx in range(len(self.seq)):
            pat = self.seq[idx]
            print('%3d %s %3d %3d %3d : (%0.3f %0.3f %0.3f)' %(idx,pat.kind,pat.axis,pat.bit,pat.inv,pat.r,pat.g,pat.b))

    def gen(self):
        self.clear_list()
        pat = 0
        if self.showinv:
            pat = MonochromePattern(1-self.r,1-self.g,1-self.b)
        else:
            pat = MonochromePattern(self.r,self.g,self.b)
        self.seq.append(pat);

    def write_list(self,outpath):
        f = open('%s/rgb_list.yaml'%outpath,'w')
        f.write("%YAML:1.0\nrgb_images:\n")
        for idx in range(len(self.seq_path)):
            f.write("   - \"%s\"\n" % self.seq_path[idx])
        if len(self.seq) != len(self.seq_path):
            f.write("rgb_images_N:%d\n"%len(self.seq));
            f.write("rgb_images_N_path:%d\n"%len(self.seq_path));

        f.close()

def send_graycode_pattern(pat):
    if pat.kind == 'gcb':
        pat.want_t = time.time()
        pat.go = True
        pat.shown = False
        tup = [int(0), int(pat.inv), int(pat.axis), int(pat.bit), float(pat.r), float(pat.g), float(pat.b)]
        print("%0.6f"%time.time(),"PATTERN REQUEST",tup)
        client_display.send( OSCMessage("/pattern_state", tup ) )
        del tup

def send_sinusoid_pattern(pat):
    if pat.kind == 'sin':
        pat.want_t = time.time()
        pat.go = True
        tup = [int(1), int(pat.inv), int(pat.axis), float(pat.freq), float(pat.phase), float(pat.r), float(pat.g), float(pat.b)]
        print("%0.6f"%time.time(),"PATTERN REQUEST",tup)
        client_display.send( OSCMessage("/pattern_state", tup ) )
        del tup

def send_monochrome_pattern(pat):
    if pat.kind == 'rgb':
        pat.want_t = time.time()
        pat.go = True
        tup = [int(2), int(pat.inv), float(pat.r), float(pat.g), float(pat.b)]
        print("%0.6f"%time.time(),"PATTERN REQUEST",tup)
        client_display.send( OSCMessage("/pattern_state", tup ) )
        del tup

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

def remote_progress_update(scan_progress, sequence_progress):
    tup = [float(scan_progress), float(sequence_progress)]
    print("%0.6f"%time.time(),"remote_progress_update",tup)
    control_server.send( OSCMessage("/progress", tup ) )
    del tup
    

def remote_turn_callback(path, tags, args, source):
    turn_dir   = min(max(int(args[0]),0),1)
    turn_steps = max(int(args[1]),1)
    print("RECEIVED REMOTE TURN EVENT (%d,%d)\n" % (turn_dir, turn_steps))
    send_turntable_command(turn_dir,turn_steps)

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

def start_capture(name):
    global sequence
    global destination
    global cap_number
    global cap_destination
    
    if sequence.go == False:
        
        # initialize pattern schedule
        if _platform == "darwin":
            Popen(['say', 'scan %s'%cap_number])
        sequence.reset()
        sequence.go = True
        sequence.start_t = time.time()

        # determine save location
        get_new_folder(name)
        sequence.name = name
        print("START @ ", sequence.start_t)
        print("*******",cap_destination, cap_number)


# args for sequence: (# of scans, # steps turn each)
def start_capture_callback(path, tags, args, source):
    global sequence

    num_scans = max(int(args[0]),1)
    num_steps = max(int(args[1]),1)
    name = str(args[2])
    print("NAME = %s\n" % name)

    del sequence
    sequence = default_sequence()
    sequence.runs = num_scans
    sequence.runs_total = num_scans
    sequence.steps = num_steps
    
    print("*** start capture: [%d] scans with [%d] steps each ***\n" % (num_scans,num_steps))

    start_capture(name)        

def cancel_capture_callback(path, tags, args, source):
    global sequence
    if sequence.go == True :
        sequence.stop_t = time.time()
        sequence.clear_list()
        if _platform == "darwin":
            Popen(["say", "sequence aborted on frame %s"%sequence.idx])
        sequence.idx = 0
        sequence.go = False
        remote_progress_update(0,0)
        print("STOP @ ", sequence.stop_t)
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
    send_graycode_pattern(use_axis,use_bit,use_inverted,use_r,use_g,use_b)
    print(use_axis,use_bit,use_inverted,use_r,use_g,use_b)

def pattern_status_callback(path, tags, args, source):
    #print("PATTERN STATUS CALLBACK",path,args)
    global ts
    global sequence

    if sequence.go:
        the_inv  = int(args[0])
        the_axis = int(args[1])
        the_bit  = int(args[2])
        the_r = float(args[3])
        the_g = float(args[4])
        the_b = float(args[5])

        idx = sequence.idx
        
        #if sequence.seq[idx].show_t < 0:
        if sequence.seq[idx].shown == False and sequence.seq[idx].go == True:
            #print("PATTERN WAS NOT SHOWN YET", sequence.seq[idx].axis, sequence.seq[idx].bit, sequence.seq[idx].inv)
            if the_axis == sequence.seq[idx].axis and the_bit == sequence.seq[idx].bit and the_inv == sequence.seq[idx].inv:
                print("PATTERN WAS MATCH")
                sequence.seq[idx].shown = True
                sequence.seq[idx].show_t = time.time()
                sequence.seq[idx].show_idx = ts.frame_n
                dt = sequence.seq[idx].show_t - sequence.seq[idx].want_t
                ts.show_dt = dt
                if ts.show_dt < ts.show_dtavg:
                    ts.show_dtsm = 0.95
                else:
                    ts.show_dtsm = 0.0
                ts.show_dtavg = ts.show_dtavg*ts.show_dtsm + ts.show_dt*(1-ts.show_dtsm)
                print("%0.6f"%sequence.seq[idx].show_t,"PATTERN SHOWN     [%d, %d, %d, %0.1f, %0.1f, %0.1f] delay = %0.6f" % (the_axis,the_bit,the_inv,the_r,the_g,the_b, dt))


def pattern_state_callback(path, tags, args, source):
    print("PATTERN STATE CALLBACK",path,args)
    global ts
    global sequence
    
    patid = int(args[0])
    match = False
    
    # GCB
    if sequence.go and patid == 0:
        the_inv  = int(args[1])
        the_axis = int(args[2])
        the_bit  = int(args[3])
        the_r  = float(args[4])
        the_g  = float(args[5])
        the_b  = float(args[6])

        idx = sequence.idx
        
        if sequence.seq[idx].shown == False and sequence.seq[idx].go == True:
            print("PATTERN WAS NOT SHOWN YET", sequence.seq[idx].axis, sequence.seq[idx].bit, sequence.seq[idx].inv)
            if the_axis == sequence.seq[idx].axis and the_bit == sequence.seq[idx].bit and the_inv == sequence.seq[idx].inv:
                match = True

    # SIN
    if sequence.go and patid == 1:
        the_inv    = int(args[1])
        the_axis   = int(args[2])
        the_freq   = int(args[3])
        the_phase  = int(args[4])
        the_r    = float(args[5])
        the_g    = float(args[6])
        the_b    = float(args[7])

        idx = sequence.idx
        
        if sequence.seq[idx].shown == False and sequence.seq[idx].go == True:
            print("PATTERN WAS NOT SHOWN YET", sequence.seq[idx].axis, sequence.seq[idx].bit, sequence.seq[idx].inv)
            if the_axis == sequence.seq[idx].axis and the_bit == sequence.seq[idx].bit and the_inv == sequence.seq[idx].inv:
                match = True

    # RGB
    if sequence.go and patid == 2:
        the_inv  = int(args[1])
        the_r  = float(args[2])
        the_g  = float(args[3])
        the_b  = float(args[4])

        idx = sequence.idx
        
        if sequence.seq[idx].shown == False and sequence.seq[idx].go == True:
            print("PATTERN WAS NOT SHOWN YET", sequence.seq[idx].axis, sequence.seq[idx].bit, sequence.seq[idx].inv)
            if the_axis == sequence.seq[idx].axis and the_bit == sequence.seq[idx].bit and the_inv == sequence.seq[idx].inv:
                match = True


    if match:
        print("PATTERN WAS MATCH")
        sequence.seq[idx].shown = True
        sequence.seq[idx].show_t = time.time()
        sequence.seq[idx].show_idx = ts.frame_n
        dt = sequence.seq[idx].show_t - sequence.seq[idx].want_t
        ts.show_dt = dt
        if ts.show_dt < ts.show_dtavg:
            ts.show_dtsm = 0.95
        else:
            ts.show_dtsm = 0.0
        ts.show_dtavg = ts.show_dtavg*ts.show_dtsm + ts.show_dt*(1-ts.show_dtsm)
        print("%0.6f"%sequence.seq[idx].show_t,"PATTERN SHOWN     [%d, %d, %d, %0.1f, %0.1f, %0.1f] delay = %0.6f" % (the_axis,the_bit,the_inv,the_r,the_g,the_b, dt))
    
    

def quit_callback(path, tags, args, source):
    global run
    run = False

#
def each_frame():
    global bytes
    global ts
    global sequence
    global cap_destination

    # do OSC server stuff
    server.timed_out = False
    while not server.timed_out:
        server.handle_request()
        
    # read mjpeg stream data
    bytes+=stream.read(81920)
    t = bytes.find('Timestamp')
    t2 = t+bytes[t:t+40].find(".")
    a = bytes.find('\xff\xd8')
    b = bytes.find('\xff\xd9')

    # parse a frame from stream and get timestamp
    jpg = ''
    frame_time = 0
    new_frame = False
    if a!=-1 and b!=-1:
        new_frame = True

        # extract image timestamp
        tstamp_s = bytes[t+11:t2]
        tstamp_ss = bytes[t2+1:t2+7]
        frame_time = float(tstamp_s) + float('0.'+tstamp_ss)
        #print("timestamp_s,ss = (%s)(%s)(%s)\n" % (bytes[t:t+50],tstamp_s, tstamp_ss))
        
        ts.new_frame(time.time(),frame_time)

        jpg = bytes[a:b+2]
        bytes= bytes[b+2:]

        dt = ts.cur_ts-ts.last_ts
        print('%0.6f'%ts.cur_framet,\
                '%0.6f late'%(ts.ts_late),\
                'frame# %0.4d'%ts.frame_n, "......",\
                'dT: %0.5f (%0.1ffps)'% ((dt,1/max(dt,0.01)) if ts.frame_n>1 else (0.0,0.0)),\
                'ts.show_dt = %f, ts.show_dtavg = %f\\n' % (ts.show_dt,ts.show_dtavg)
                )
                #'show_idx[i] = %f, show_idx[i-1] = %f\n'%(sequence.seq[max(sequence.idx,0)].show_idx,sequence.seq[max(sequence.idx-1,0)].show_idx ))
                #'show_t[i] = %f, show_t[i-1] = %f\n'%(sequence.seq[max(sequence.idx,0)].show_t,sequence.seq[max(sequence.idx-1,0)].show_t ))



    # save new image after confirmation of pattern display
    if new_frame and sequence.go:
        #conditions to save current frame
        cond_framedelay = (ts.frame_n > sequence.seq[max(sequence.idx,0)].show_idx + 4)
        cond_timedelay  = (ts.cur_ts > sequence.seq[max(sequence.idx,0)].show_t + 0.2 )
        #cond_timedelay  = (ts.cur_ts > sequence.seq[max(sequence.idx,0)].show_t + max(ts.dt,max(ts.show_dt,ts.show_dtavg)) + 0.1 )

        # ensure image is taken after pattern change
        if sequence.seq[sequence.idx].go and not sequence.seq[sequence.idx].saved and\
                sequence.seq[sequence.idx].shown and sequence.seq[sequence.idx].show_idx > 0 and\
                cond_framedelay and cond_timedelay:
            
            seqP = (sequence.runs_total-sequence.runs) / float(max(sequence.runs_total-1,1))
            scanP = sequence.idx / float(len(sequence.seq)-1)
            remote_progress_update(scanP,seqP)

            i = cv2.imdecode(np.fromstring(jpg, dtype=np.uint8),cv2.IMREAD_COLOR)
            pat = sequence.seq[sequence.idx]
            #frame_datetime = datetime.fromtimestamp(frame_time)

            fname = '%0.6f_[%s_%02d]_img.jpg' % (ts.cur_ts, '%02d-%02d-%02d'%(pat.axis, pat.bit, pat.inv), sequence.idx )
            outpath = '%s/%s' % (cap_destination, fname)
            
            print(" writing : [%s]\n"%outpath)
            cv2.imwrite(outpath,i)
            print('%0.6f'%ts.cur_ts, 'frame# %0.4d'%ts.frame_n, "saved [IDX %d]"%sequence.idx, '%0.6f s after display'%(ts.cur_ts-sequence.seq[sequence.idx].show_t))
            print("*******************************************************************************************************")
            #cv2.imwrite('%s/%0.6f_[%s]_img.jpg' % (cap_destination, ts.cur_ts, frame_datetime.strftime("%Y-%m-%d_%H-%M-%S-%f")),i)
            
            # add filename to sequence
            sequence.commit_image(fname,ts.frame_n,ts.cur_framet)

            # sequence termination logic
            if sequence.idx >= len(sequence.seq):
                sequence.write_list(cap_destination)
                sequence.clear_list()
                #frame_time = len(sequence.seq)-1
                sequence.stop_t = time.time()
                scanduration = (sequence.stop_t-sequence.start_t)
                sequence.runs = max(sequence.runs-1,0)
                print("SEQUENCE DURATION: %0.3f seconds"%scanduration)
                sequence.go = False
                if _platform == "darwin":
                    Popen(["say", "scan %d completed in %d seconds"%(cap_number,int(scanduration))])
                
                # more scans to do
                if sequence.runs > 0:
                    print("********* TURNTABLE EVENT *********\n")
                    send_turntable_command(1,sequence.steps)
                    sleep(5)
                    start_capture(sequence.name)
                # all done
                else:
                    sleep(5)
                    if _platform == "darwin":
                        Popen(["say", "sequence terminated"])
                    # Popen(["slcrunch", "-b 1 -c"])
                    # process all the scan data we just saved
                    # shell script 
            else:
                sequence.seq[sequence.idx].show_t = -1
        
    # send pattern for display
    #if sequence.seq[max(0,sequence.idx-1)].save_t + 0.3 < time.time() and sequence.seq[sequence.idx].go == False:
    if sequence.go and len(sequence.seq) > 0 and sequence.idx < len(sequence.seq):
        if sequence.idx == 0 or (sequence.idx > 0 and sequence.seq[max(0,sequence.idx-1)].save_t + 2/60.0 < time.time()):
            if sequence.seq[sequence.idx].go == False:
                send_graycode_pattern(sequence.seq[sequence.idx])
                send_graycode_pattern(sequence.seq[sequence.idx])
                #image_N_lastchange = ts.frame_n
                #pat = sequence.seq[sequence.idx]
    
    # send pattern
    if sequence.go and len(sequence.seq) > 0 and sequence.idx < len(sequence.seq):
        if sequence.idx == 0 or (sequence.idx > 0 and sequence.seq[max(0,sequence.idx-1)].save_t + 2/60.0 < time.time()):
            if sequence.seq[sequence.idx].go == True and sequence.seq[sequence.idx].shown == False and sequence.seq[sequence.idx].want_t < time.time()-3:
                send_graycode_pattern(sequence.seq[sequence.idx])

    #print("(\'%0.6f idx=%d"%(time.time(),sequence.idx))
    #cv2.imshow('i',i)
    #if cv2.waitKey(1) == 27:
    #    exit(0)

def default_sequence():
    v = 1.0
    return GrayCodeBinaryPatternSequence(11,True,True,True,v,v,v)

#--------------------------------------------------------------------------#

def cleanup():
    global turntable_enabled
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

if __name__ == "__main__":
    global turntable_enabled

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
    stream=urllib.urlopen('http://%s:8080/?action=stream' % cam_server_addr)
    bytes=''

    ts = TimeState(time.time())

    #image_N = 0
    #image_N_lastchange = 0
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

    # sequence trigger vars
    sequence = []

    server.addMsgHandler( "/turn", remote_turn_callback )
    server.addMsgHandler( "/start_capture", start_capture_callback )
    server.addMsgHandler( "/cancel_capture", cancel_capture_callback )
    server.addMsgHandler( "/manual/axis", manual_pattern_callback )
    server.addMsgHandler( "/manual/bit", manual_pattern_callback )
    server.addMsgHandler( "/manual/inv", manual_pattern_callback )
    server.addMsgHandler( "/manual/r", manual_color_callback )
    server.addMsgHandler( "/manual/g", manual_color_callback )
    server.addMsgHandler( "/manual/b", manual_color_callback )
    server.addMsgHandler( "/pattern_status", pattern_status_callback )
    #server.addMsgHandler( "/pattern_state", pattern_state_callback )
    #server.addMsgHandler( "/quit", quit_callback )

    sequence = default_sequence()

    itr_t = time.time()
    itr_t_last = time.time()

    run = True
    # loop forever until quit
    while run:
        sleep(1/60.0)
        each_frame()

    cleanup()
