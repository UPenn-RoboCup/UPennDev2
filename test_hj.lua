dofile'fiddle.lua'
body_ch:send'init'
local ffi = require "ffi"
ffi.cdef "unsigned int sleep(unsigned int seconds);"
ffi.C.sleep(10)
Body.set_larm_command_position({0,0,0,0,0,0,0})
Body.set_rarm_command_position({0,0,0,0,0,0,0})

