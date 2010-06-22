#!/usr/bin/python

import subprocess
import time
import datetime

class Pentemp:
    def __init__(self):
        self.device = '/dev/pentek/p7142/0/t1'
        self.cmd = ['drvgetparm', self.device]
        
    def doCmd(self):
        cmd = subprocess.Popen(self.cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        lines = cmd.stdout.readlines()
        temps = []
        for l in lines:
            words = l.split()
            if words[0] == 'Measured':
                temps.append(words[-1])
        return temps
        
    @staticmethod
    def run():
        pentemp = Pentemp()
        i = 0
        while 1:
            temps = pentemp.doCmd()
            print i,
            for t in temps:
                print t,
            print
            i = i + 1
            time.sleep(10)


if __name__ == '__main__':
    Pentemp.run()
