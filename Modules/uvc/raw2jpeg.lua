dofile'../../include.lua'
msgpack=require'msgpack'

date = '04.09.2014.14.59.58'
DIR = '/Users/smcgill3/Box Sync/YouBot'

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

f_r = io.open(DIR..'/uvc_r_'..date..'.log','r')
jpeg = require'jpeg'
cy = jpeg.compressor('yuyv')
cy:downsampling(0)
for i,m in ipairs(metadata) do
	local img = f_r:read(m.sz)
	print('image size',#img)
	local jimg = cy:compress(img, 320, 240)
	f_i = io.open('/tmp/img'..i..'.jpeg','w')
	f_i:write(jimg)
	f_i:close()
end
f_r:close()
