dofile'fiddle.lua'
head_ch:send''
unix.usleep(3e6)
motion_ch:send'hybridwalk'
unix.usleep(1e6)
mcm.set_walk_kicktype(1)
body_ch:send'approach'
