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
local y_plane
for i,m,r in d do
	if i>#metadata/2 then break end
	local t0 = unix.time()
	meta = m
	yuyv_t = r
	local w, h = meta.w, meta.h

  -- Format the label image
  labelA:resizeAs(torch.ByteTensor(w/2,h/2))
  -- Get the subsample information for strides
  y_plane = yuyv_t:reshape(h/2,w,4):sub(1,-1,1,w/2):select(3,1)
  local y = y_plane:cdata()
	local na, nb = tonumber(y.size[0])-1, tonumber(y.size[1])-1
  -- Counters on the planes
  local i, s, idx = 0, tonumber(y.stride[0]), tonumber(y.storageOffset)
  local lA, yuyv = labelA:data(), yuyv_t:data()
	local y6,u6,v6,index,cdt
  for a=0,na do
    for b=0,nb do
      y6, u6, v6 =
        rshift(yuyv[idx], 2),
        band(yuyv[idx+1],0xFC),
        band(yuyv[idx+2],0xFC)
			index = bor( y6, lshift(u6,4), lshift(v6,10) )
      cdt = lut[index]
			lA[i] = cdt
      -- Stride to next, assume 4
      idx = idx + 4
			i = i + 1
    end
    -- stride to next
    idx = idx + s
  end

	local t1 = unix.time()
	print('Processing Time',t1-t0)
end
-- Save to file
print('labelA',labelA:size(1),labelA:size(2))
local f_l = torch.DiskFile('labelA.raw', 'w')
f_l.binary(f_l)
f_l:writeByte(labelA:storage())
f_l:close()

local jpeg = require'jpeg'
c_gray = jpeg.compressor('gray')
--
local str = c_gray:compress(y_plane:clone())
local f_y = io.open('y_plane.jpeg','w')
f_y:write(str)
f_y:close()
