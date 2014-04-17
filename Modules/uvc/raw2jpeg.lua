dofile'../../include.lua'
msgpack=require'msgpack'


--date = '04.09.2014.14.59.58'
--date = '04.17.2014.15.00.52'
date = '04.17.2014.16.34.17'
DIR = '/Users/smcgill3/Box Sync/YouBot/log2'

-- Read the metadata
local m_name = DIR..'/uvc_m_'..date..'.log'
print(m_name)
f_m = io.open(m_name,'r')
-- Must use an unpacker...
local metadata = {}
local unpacker = msgpack.unpacker(2048)
local buf, nbuf = f_m:read(512),0
while buf do
	nbuf = nbuf + #buf
	local res,left = unpacker:feed(buf)
	local tbl = unpacker:pull()
	while tbl do
		metadata[#metadata+1] = tbl
		tbl = unpacker:pull()
	end
	buf = f_m:read(left)
end
f_m:close()

print('Unlogging',#metadata,'images')

util=require'util'

f_r = io.open(DIR..'/uvc_r_'..date..'.log','r')
jpeg = require'jpeg'
cy = jpeg.compressor('yuyv')
cy:downsampling(0)
for i,m in ipairs(metadata) do
	--util.ptable(m)
	print('arm',unpack(m.arm))
	local img = f_r:read(m.rsz)
	print('image size',#img,m.sz)
	local jimg = cy:compress(img, m.w, m.h)
	local fn = string.format('/tmp/img%04d.jpeg',i)
	f_i = io.open(fn,'w')
	f_i:write(jimg)
	f_i:close()
end
f_r:close()
