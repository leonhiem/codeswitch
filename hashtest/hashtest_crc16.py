#!/usr/bin/python

import uuid
import hashlib
import crc16

crc_list=[]
collision_cnt=0
cnt=1000

for i in range(0,cnt):
    myid=uuid.uuid4().int
    crc=crc16.crc16xmodem(str(myid))
    if crc in crc_list:
        print "already in list. UUID=",myid," CRC=", crc
        collision_cnt+=1
    crc_list.append(crc)

print "nof collisions:",collision_cnt, "(",float(collision_cnt)/float(cnt)*100.0,"%)"


