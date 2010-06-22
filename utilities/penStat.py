#!/usr/bin/python
#

import os

upDev="/dev/pentek/p7142/0/up/0"

allIoctls=[
"CLKSRCGET",
"SAMPRATEGET",
"BUFGET",
"GETOVRCNT",
"RSFILRNDGET",
"RSFILGET",
"RSGETRATIO",
"INCHANGET",
"NCOGET",
"SPECTFORMGET",
"ADM1024GET",
"GETPHASE",
"GETOVRCNT",
"DECIMGET",
"BYPDIVGET",
"DUCMDGET",
"PLLDIVGET",
"PLLVDDGET",
"DACCLKGET",
"SYNCBUSGET",
]

upIoctls=[
"CLKSRCGET",
"SAMPRATEGET",
"GETOVRCNT",
"NCOGET",
"GETPHASE",
"GETOVRCNT",
"DECIMGET",
"PLLDIVGET",
"PLLVDDGET",
"DACCLKGET",
"SYNCBUSGET",
]

for i in upIoctls:
    cmd='drvgetparm ' + upDev + ' ' + i
    pipe = os.popen(cmd)
    result = pipe.read()
    result = result[:-1]
    print result

cmd='drvmon ' + upDev + ' 0'
(pin,pout) = os.popen2(cmd, 'rw')
pin.write('\n')
pin.flush()
print pout.readline()
for i in range(1,8):
    pass
    
    
