#!/usr/bin/python

import uuid
import zlib

crc_list=[]
collision_cnt=0
cnt=100000

#n=232280341883203880131143036734844459601

for i in range(0,cnt):
    myid=uuid.uuid4().int
    #n+=1
    #myid=str(n)

    crc=zlib.crc32(str(myid))

    #print "UUID=", myid, "\tcrc=",crc

    if crc in crc_list:
        print "already in list. UUID=",myid," CRC=", crc
        collision_cnt+=1
    crc_list.append(crc)

print "cnt=",cnt
print "nof collisions:",collision_cnt, "(",float(collision_cnt)/float(cnt)*100.0,"%)"


