#!/usr/local/opt/python/libexec/bin/python
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
from OSC import OSCServer, OSCClient, OSCMessage, OSCStreamingClient
from subprocess import call
from subprocess import Popen
from sys import platform as _platform
from time import sleep
import copy

# track frame timing
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

    lag_checknow = True
    lag_iterations = 5
    lag_avgdark = 0
    lag_avglight = 0
    lag_integrationtime = 10

    img_size_last = 1650000
    img_size_avg = 1650000
    img_size_avg_sm = 0.1

    def __init__(self,now):
        start = now

    def new_frame(self,now,timestamp,size):
        self.frame_n = self.frame_n + 1
        self.last_framet = self.cur_framet
        self.cur_framet = now
        self.last_ts = self.cur_ts
        self.cur_ts = timestamp

        self.ts_late = self.cur_framet-self.cur_ts
        self.ts_lateavg = self.ts_lateavg*self.ts_latesm + self.ts_late*(1-self.ts_latesm)

        self.dt = self.cur_ts-self.last_ts
        self.dt_avg = self.dt_avg*self.dt_sm + self.dt*(1-self.dt_sm)
        
        self.img_size_last = size
        
        diff = abs(self.img_size_avg-self.img_size_last)

        if diff > 1000:
            self.img_size_avg_sm = 0.85
        if diff > 5000:
            self.img_size_avg_sm = 0.1
        if diff > 10000:
            self.img_size_avg_sm = 0.01

        self.img_size_avg = self.img_size_avg*self.img_size_avg_sm + (1-self.img_size_avg_sm)*self.img_size_last


##################
# PATTERN TYPES
# generic pattern specification type
class StructuredLightPattern(object):
    kind = 'pat'
    name = ''
    
    inv = 0
    r = 1
    g = 1
    b = 1

    go    = False
    shown = False
    saved = False
    
    go_t   = -1
    show_t = -1
    save_t = -1
    
    show_idx = -1
    dt = -1

    def __init__(self):
        pass

    def setcolor(self,r,g,b):
        self.r = r
        self.g = g
        self.b = b

    def setname(name):
        self.name = name

# pattern = x or y axis gray code binary single bitfield mapped to luminance with rgb weights
class GrayCodeBinaryPattern(StructuredLightPattern):
    axis = 0
    bit  = 0

    def __init__(self,inv,axis,bit):
        self.kind = 'gcb'
        self.inv  = inv
        self.axis = axis
        self.bit  = bit

    def tostr(self):
        ret = ""
        ret += self.kind + ", "
        ret += self.inv + ", "
        ret += self.r + ", "
        ret += self.g + ", "
        ret += self.b + ", "
        ret += self.axis + ", "
        ret += self.bit
        return ret

# pattern = x or y axis sine function mapped to luminance with rgb weights
class SinusoidPattern(StructuredLightPattern):
    axis  = 0
    freq  = 1
    phase = 0
    
    def __init__(self,inv,axis,freq,phase):
        self.kind = 'sin'
        self.inv  = inv
        self.axis = axis
        self.freq = freq
        self.phase = phase

    def tostr(self):
        ret = ""
        ret += self.kind + ", "
        ret += self.inv + ", "
        ret += self.r + ", "
        ret += self.g + ", "
        ret += self.b + ", "
        ret += self.axis + ", "
        ret += self.freq + ", "
        ret += self.phase
        return ret

# pattern = rgb triplet applied to every pixel
class MonochromePattern(StructuredLightPattern):
    def __init__(self,inv,r,g,b):
        self.kind = 'rgb'
        self.inv = inv
        self.r = r
        self.g = g
        self.b = b

    def tostr(self):
        ret = ""
        ret += self.kind + ", "
        ret += self.inv + ", "
        ret += self.r + ", "
        ret += self.g + ", "
        ret += self.b
        return ret
    

#########################
# SEQUENCE BUFFER TYPE
# collection of sequences to be batched together
# sequence of sequences of patterns
class SequenceBuffer(object):
    kind = 'seqbuf'
    name = ''
    
    go = False
    completed = False    
    start_t = -1
    stop_t = -1

    runs_total = 1
    runs = 1
    steps = 50

    idx = 0
    seq = []

    def __init__(self,outname):
        self.name = outname
    
    def is_empty(self):
        if len(seq) == 0:
            return True
        return False
    
    def should_go(self):
        if self.go and len(self.seq) > 0 and self.idx < len(self.seq):
            return self.seq[self.idx].idx < len(self.seq[self.idx].pat)
            return True
            #return self.seq[self.idx].go
        else:
            return False
    
    def num_pats(self):
        count = 0
        for sub_seq in self.seq:
            count += len(sub_seq.pat)
        return count

    # wipe structure
    def reset(self):
        del self.seq[:]
        self.seq = []
        go = False
        completed = False
        name = ''
        start_t = -1
        stop_t = -1
        runs_total = 1
        runs = 1
        steps = 0

    def add_sequence(self,newseq):
        #self.seq.append(copy.deepcopy(newseq))
        self.seq.append(newseq)
    
    def write_list(self,outpath):
        f = open('%s/sequence.yaml'%outpath,'w')
        f.write("%YAML:1.0\n")
        for sub_seq in self.seq:
            f.write( sub_seq.generate_raw_yaml() )

    def log(self):
        ret = ''
        i = 0
        #ret += ("len(seq) = %d" % len(self.seq))
        for sub_seq in self.seq:
            #print(i,sub_seq.generate_raw_yaml())
            ret += "[ %s with %d %s patterns ]\n" % (sub_seq.kind, len(sub_seq.pat), sub_seq.pat[0].kind)
            i += 1
        return ret

#########################
# SEQUENCE TYPES
# container to hold sequence of patterns
class PatternSequence(object):
    kind = 'seq'
    name = ''

    showinv = False    

    go = False
    completed = False
    start_t = -1
    stop_t = -1
    
    r = 1.0
    g = 1.0
    b = 1.0
    
    idx = 0
    pat = []
    pat_path = []

    def __init__(self):
        self.kind = 'seq'
        pass

    def gen():
        pass
    
    def is_primed(self,delay):
        if len(self.pat) > 0:
            if self.idx == 0 or (self.idx > 0 and self.pat[max(0,self.idx-1)].save_t + delay < time.time()):
                return True
        else:
            return False
                #return self.pat[self.idx].go

    def commit_image(self,path,newidx,newframetime):
        self.pat_path.append(path)
        self.pat[self.idx].saved = True
        self.pat[self.idx].save_t = time.time()
        self.pat[self.idx].show_t = newframetime
        #self.pat[self.idx].show_idx = newidx
        self.idx += 1

    def clear_list(self):
        del self.pat[:]
        del self.pat_path[:]
        self.pat = []
        self.pat_path = []
    
    def reset(self):
        self.go = False
        self.completed = False
        self.start_t = -1
        self.stop_t = -1
        self.idx = 0
        self.gen()
 
class GrayCodeBinaryPatternSequence(PatternSequence):
    bits = 0
    showx = True
    showy = False
    frame_mult = 1
    
    def __init__(self,bits,showx,showy,showinv,rv,gv,bv):
        self.kind = 'gcb_seq'
        self.bits = bits
        self.showx = showx
        self.showy = showy
        self.showinv = showinv
        self.r = rv
        self.g = gv
        self.b = bv
        self.gen()
        frame_mult = 2 if (showx and showy) else 1
        frame_mult *= 2 if showinv else 1
    
    def print_debug(self):
        for idx in range(len(self.pat)):
            pat = self.pat[idx]
            print('%3d %s %3d %3d %3d : (%0.3f %0.3f %0.3f)' %(idx,pat.kind,pat.axis,pat.bit,pat.inv,pat.r,pat.g,pat.b))

    def gen(self):
        self.clear_list()
        if self.showx:
            for x in range(self.bits):
                idx = self.bits-x-1
                pat = GrayCodeBinaryPattern(0,0,idx)
                pat.setcolor(self.r,self.g,self.b)
                self.pat.append(pat)
                if self.showinv:
                    patinv = GrayCodeBinaryPattern(1,0,idx)
                    patinv.setcolor(self.r,self.g,self.b)
                    self.pat.append(patinv)
        if self.showy:
            for y in range(self.bits):
                idx = self.bits-y-1
                pat = GrayCodeBinaryPattern(0,1,idx)
                pat.setcolor(self.r,self.g,self.b)
                self.pat.append(pat)
                if self.showinv:
                    patinv = GrayCodeBinaryPattern(1,1,idx)
                    patinv.setcolor(self.r,self.g,self.b)
                    self.pat.append(patinv)
    
    def generate_raw_yaml(self):
        ret = ""
        ret += "gcb_images:\n"
        for idx in range(len(self.pat_path)):
            ret += str("   - \"%s\"\n" % self.pat_path[idx])

        ret += "gcb_param:\n"
        ret += str("   - \"%s\"\n" % self.bits)
        ret += str("   - \"%s\"\n" % self.showx)
        ret += str("   - \"%s\"\n" % self.showy)
        ret += str("   - \"%s\"\n" % self.showinv)
        ret += str("   - \"%s\"\n" % self.r)
        ret += str("   - \"%s\"\n" % self.g)
        ret += str("   - \"%s\"\n" % self.b)
        
        #ret += "gcb_patterns:\n"
        #for pat in self.pat:
        #    ret += str("   - \"%s\"n" % pat.axis)

        #for idx in range(len(self.pat)):
        #    ret += str("   - \"%s\"\n" % self.pat[idx].axis)

        #if len(self.pat) != len(self.pat_path):
        #    ret += str("gcb_images_N:%d\n"%len(self.pat))
        #    ret += str("gcb_images_N_path:%d\n"%len(self.pat_path))
        return ret

    def write_list(self,outpath):
        f = open('%s/list.yaml'%outpath,'w')
        f.write( "%YAML:1.0\n" )
        f.write( self.generate_raw_yaml() )        
     
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
    frames = 3 
    showx = True
    showy = False

    def __init__(self,frames,showx,showy,showinv,r,g,b):
        self.kind = 'sin_seq'
        self.frames = frames
        self.showx = showx
        self.showy = showy
        self.showinv = showinv
        self.r = r
        self.g = g
        self.b = b
        self.reset()
        frame_mult = 2 if (showx and showy) else 1
        frame_mult *= 2 if showinv else 1

    def print_debug(self):
        for idx in range(len(self.pat)):
            pat = self.pat[idx]
            print('%3d %s %3d %3d %3d : (%0.3f %0.3f %0.3f)' %(idx,pat.kind,pat.axis,pat.freq,pat.phase,pat.r,pat.g,pat.b))

    def gen(self):
        self.clear_list()
        pat = 0
        if self.showinv:
            pat = SinusoidPattern(1-self.r,1-self.g,1-self.b)
        else:
            pat = SinusoidPattern(self.r,self.g,self.b)
        self.pat.append(pat)

    def generate_raw_yaml(self):
        ret = "sin_images:\n"
        for idx in range(len(self.pat_path)):
            ret += str("   - \"%s\"\n" % self.pat_path[idx])
        if len(self.pat) != len(self.pat_path):
            ret += str("sin_images_N:%d\n"%len(self.pat))
            ret += str("sin_images_N_path:%d\n"%len(self.pat_path))
        return ret
        
    def write_list(self,outpath):
        f = open('%s/sin_list.yaml'%outpath,'w')
        f.write( "%YAML:1.0\n" )
        f.write( self.generate_raw_yaml() )

class MonochromePatternSequence(PatternSequence):

    def addColor(self,inv,r,g,b):
        pat = MonochromePattern(inv,r,g,b)
        self.pat.append(pat)
    
    def addColor(self,r,g,b):
        pat = MonochromePattern(False,r,g,b)
        self.pat.append(pat)

    def __init__(self):
        self.showinv = False

    def __init__(self,inv,r,g,b):
        self.kind = 'rgb_seq'
        self.showinv = inv
        self.r = r
        self.g = g
        self.b = b
        self.reset()

    def print_debug(self):
        for idx in range(len(self.pat)):
            pat = self.pat[idx]
            print('%3d %s %3d : (%0.3f %0.3f %0.3f)' %(idx,pat.kind,pat.inv,pat.r,pat.g,pat.b))

    def gen(self):
        self.clear_list()
        pat = 0
        if self.showinv:
            pat = MonochromePattern(False,1-self.r,1-self.g,1-self.b)
        else:
            pat = MonochromePattern(False,self.r,self.g,self.b)
        self.pat.append(pat)

    def generate_raw_yaml(self):
        patN = len(self.pat)
        ret = "rgb_images:\n"
        for idx in range(len(self.pat_path)):
            ret += str("   - \"%s\"\n" % self.pat_path[idx])
        if patN != len(self.pat_path):
            ret += str("rgb_images_N:%d\n" % patN)
            ret += str("rgb_images_N_path:%d\n"%len(self.pat_path))
        ret += "rgb_values: !!opencv-matrix\n"
        ret += "   rows: %d\n" % patN
        ret += "   cols: 1\n"
        ret += "   dt: \"3f\"\n"
        ret += "   data: [ "
        for idx in range(patN):
            ret += "%0.6f, %0.6f, %0.6f" % ( self.pat[idx].r, self.pat[idx].g, self.pat[idx].b )
            if idx < patN-1:
                ret += ",\n           "
        ret += " ]"


        return ret

    def write_list(self,outpath):
        print self.generate_raw_yaml()
        f = open('%s/rgb_list.yaml'%outpath,'w')
        f.write( "%YAML:1.0\n" )
        f.write( self.generate_raw_yaml() )        

