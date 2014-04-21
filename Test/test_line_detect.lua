dofile'../include.lua'
local libLog = require'libLog'

date = '04.17.2014.16.34.17'
DIR = HOME..'/Logs/'
local replay = libLog.open(DIR,date)
local metadata = replay:unroll_meta()
print('Unlogging',#metadata,'images')

local d = replay:log_iter(metadata)
local meta, yuyv = d()
print('Point',meta,type(yuyv))

-- Extract the Y-plane
-- Assume LuaJIT right now...
print('yuyv',yuyv,meta.w,meta.h)
local y_plane = ffi.new('uint8_t[?]',meta.w*meta.h)
for i=1,120 do
	io.write(yuyv[i],'\t')
	if i%4==0 then print() end
end
