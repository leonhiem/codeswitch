#!/usr/bin/python

import uuid
import hashlib

md5_list=[]
collision_cnt=0
cnt=100000


for i in range(0,cnt):
    myid=uuid.uuid4().int

    m=hashlib.md5()
    m.update(str(myid))
    md5=m.hexdigest()
    del m

    # truncate md5:
    md5_ = md5[:8]
    #print "UUID=", myid, "\tmd5=",md5, "md5_=",md5_

    if md5_ in md5_list:
        print "already in list. UUID=",myid," md5_=", md5_
        collision_cnt+=1

    md5_list.append(md5_)

print "cnt=",cnt
print "nof collisions:",collision_cnt, "(",float(collision_cnt)/float(cnt)*100.0,"%)"


