dofile'../include.lua'
local libLog = require'libLog'
local torch = require'torch'
local util = require'util'
local bit = require'bit'
local lshift = bit.lshift
local rshift = bit.rshift
local bor = bit.bor
local band = bit.band

date = '04.17.2014.16.34.17'
DIR = HOME..'/Logs/'
local replay = libLog.open(DIR,date)
local metadata = replay:unroll_meta()
print('Unlogging',#metadata,'images')

local d = replay:log_iter(metadata)

-- TODO: Convolution placeholder for speed
local meta, yuyv, yuyv_sub, y_plane, u_plane, v_plane
-- Load the LUT into a torch object
local lut_name = HOME.."/Data/lut_NaoV4_Grasp.raw"
--print("lut_name",lut_name)
local f_lut = torch.DiskFile(lut_name, 'r')
--print('Opened',f_lut)
f_lut.binary(f_lut)
-- We know the size of the LUT
local lut_s = f_lut:readByte(262144)
f_lut:close()
local lut = lut_s:data()

local labelA = torch.ByteTensor()

for i,m,r in d do
	local t0 = unix.time()
	meta = m
	yuyv = r
	local w, h = meta.w, meta.h
	-- Get the sub_sampled planes
	yuyv_sub = r:reshape(h/2,w,4):sub(1,-1,1,w/2)
	-- Get the y-plane
	-- Use LuaJIT and have only in the cdata format
	y_plane = yuyv_sub:select(3,1):data()
	u_plane = yuyv_sub:select(3,2):data()
	v_plane = yuyv_sub:select(3,3):data()
	-- Construct the label image
	labelA:resizeAs(torch.ByteTensor(w/2,h/2))
	local lA = labelA:data()
	for i=0, labelA:nElement()-1 do
		--[[
		local y6, u6, v6 = 
		rshift(y_plane[i], 2),
		band(u_plane[i],0xFC),
		band(v_plane[i],0xFC)
		local idx = bor( y6, lshift(u6,4), lshift(v6,10) )
		lA[i] = lut[idx]
		--]]
		lA[i] = bor(
			rshift(y_plane[i], 2),
			lshift(band(u_plane[i],0xFC),4),
			lshift(band(v_plane[i],0xFC),10)
		)
	end
	--y1_plane = yuyv_sub:select(3,4)
	local t1 = unix.time()
	print('Processing Time',t1-t0)
end

