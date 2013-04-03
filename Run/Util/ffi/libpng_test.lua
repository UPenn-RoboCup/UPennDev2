local glue = require'glue'
local ffi = require'ffi'
local bmpconv = require'bmpconv'
local pp = require'pp'.pp
local readfile = glue.readfile
require'unit'
local libpng = require'libpng'

for _,filename in ipairs(dir('media/png/good/*.png')) do
	print(filename,'----------------------------')
	local s = readfile(filename)
	local cdata = ffi.new('unsigned char[?]', #s+1, s)

	pp(libpng.load({path = filename}))
	pp(libpng.load({cdata = cdata, size = #s}))

	for _,row_format in ipairs{'top_down', 'bottom_up'} do
		for _,padded in ipairs{true, false} do
			for _,pixel_format in ipairs{'g', 'ga', 'ag', 'rgb', 'bgr', 'rgba', 'argb', 'bgra', 'abgr'} do
				print('>', pixel_format, row_format, padded)
				local t = libpng.load({string = readfile(filename)},
						{accept = {[pixel_format] = true, [row_format] = true, padded = padded}})
				pp(t)
				assert(t.stride == padded and bmpconv.pad_stride(#t.pixel * t.w) or #t.pixel * t.w)
				assert(t.size == t.stride * t.h)
			end
		end
	end
end
