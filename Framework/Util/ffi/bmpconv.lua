--pixel format upsampling, resampling and downsampling for luajit.
--supports all conversions between packed 8 bit-per-channel gray and rgb pixel formats + cmyk to all.
--supports different input/output orientations, namely top-down and bottom-up, and different strides.
--can split up the conversion to multiple threads taking advantage of multiple cpu cores.

--conversion functions: these must run in lanes, so don't drag any upvalues with them

local ffi = require'ffi' --for lanes

local function dstride(src, dst)
	local dj, dstride = 0, dst.stride
	if src.orientation ~= dst.orientation then
		dj = (src.h - 1) * dstride --first pixel of the last row
		dstride = -dstride --...and stepping backwards
	end
	return dj, dstride
end

local function eachrow(convert)
	return function(src, dst, h1, h2)
		--local ffi = require'ffi' --for lanes
		local dj, dstride = dstride(src, dst)
		local pixelsize = #src.pixel
		local rowsize = src.w * pixelsize
		local src_data = ffi.cast('uint8_t*', src.data) --ensure byte type (also src.data is a number when in lanes)
		local dst_data = ffi.cast('uint8_t*', dst.data)
		for sj = h1 * src.stride, h2 * src.stride - 1, src.stride do
			convert(dst_data, dj, src_data, sj, rowsize)
			dj = dj + dstride
		end
	end
end

local copy_rows = eachrow(function(d, i, s, j, rowsize)
	--local ffi = require'ffi' --for lanes
	ffi.copy(d+i, s+j, rowsize)
end)

local function eachpixel(convert)
	return function(src, dst, h1, h2)
		--local ffi = require'ffi' --for lanes
		local dj, dstride = dstride(src, dst)
		local pixelsize = #src.pixel
		local dpixelsize = #dst.pixel
		local rowsize = src.w * pixelsize
		local src_data = ffi.cast('uint8_t*', src.data)
		local dst_data = ffi.cast('uint8_t*', dst.data)
		for sj = h1 * src.stride, h2 * src.stride - 1, src.stride do
			for si = 0, rowsize - 1, pixelsize do
				convert(dst_data, dj + si * dpixelsize, src_data, sj + si)
			end
			dj = dj + dstride
		end
	end
end

--pixel conversion functions: these must also run in lanes

local matrix = {
	g = {},
	ga = {}, ag = {},
	rgb = {}, bgr = {},
	rgba = {}, bgra = {}, argb = {}, abgr = {},
	cmyk = {},
}

matrix.ga.ag = eachpixel(function(d, i, s, j) d[i], d[i+1] = s[j+1], s[j] end)
matrix.ag.ga = matrix.ga.ag

matrix.bgr.rgb = eachpixel(function(d, i, s, j) d[i], d[i+1], d[i+2] = s[j+2], s[j+1], s[j] end)
matrix.rgb.bgr = matrix.bgr.rgb

matrix.rgba.abgr = eachpixel(function(d, i, s, j) d[i], d[i+1], d[i+2], d[i+3] = s[j+3], s[j+2], s[j+1], s[j+0] end)
matrix.bgra.argb = matrix.rgba.abgr
matrix.argb.bgra = matrix.rgba.abgr
matrix.abgr.rgba = matrix.rgba.abgr
matrix.argb.rgba = eachpixel(function(d, i, s, j) d[i], d[i+1], d[i+2], d[i+3] = s[j+1], s[j+2], s[j+3], s[j+0] end)
matrix.abgr.bgra = matrix.argb.rgba
matrix.rgba.argb = eachpixel(function(d, i, s, j) d[i], d[i+1], d[i+2], d[i+3] = s[j+3], s[j+0], s[j+1], s[j+2] end)
matrix.bgra.abgr = matrix.rgba.argb
matrix.rgba.bgra = eachpixel(function(d, i, s, j) d[i], d[i+1], d[i+2], d[i+3] = s[j+2], s[j+1], s[j+0], s[j+3] end)
matrix.bgra.rgba = matrix.rgba.bgra
matrix.argb.abgr = eachpixel(function(d, i, s, j) d[i], d[i+1], d[i+2], d[i+3] = s[j+0], s[j+3], s[j+2], s[j+1] end)
matrix.abgr.argb = matrix.argb.abgr

matrix.g.ag = eachpixel(function(d, i, s, j) d[i+1], d[i+0] = s[j], 0xff end)
matrix.g.ga = eachpixel(function(d, i, s, j) d[i+0], d[i+1] = s[j], 0xff end)

matrix.g.rgb = eachpixel(function(d, i, s, j) d[i], d[i+1], d[i+2] = s[j], s[j], s[j] end)
matrix.g.bgr = matrix.g.rgb

matrix.g.argb = eachpixel(function(d, i, s, j) d[i], d[i+1], d[i+2], d[i+3] = 0xff, s[j], s[j], s[j] end)
matrix.g.abgr = matrix.g.argb
matrix.g.rgba = eachpixel(function(d, i, s, j) d[i], d[i+1], d[i+2], d[i+3] = s[j], s[j], s[j], 0xff end)
matrix.g.bgra = matrix.g.rgba

matrix.ga.rgba = eachpixel(function(d, i, s, j) d[i], d[i+1], d[i+2], d[i+3] = s[j+0], s[j+0], s[j+0], s[j+1] end)
matrix.ga.bgra = matrix.ga.rgba
matrix.ga.argb = eachpixel(function(d, i, s, j) d[i], d[i+1], d[i+2], d[i+3] = s[j+1], s[j+0], s[j+0], s[j+0] end)
matrix.ga.abgr = matrix.ga.argb
matrix.ag.rgba = eachpixel(function(d, i, s, j) d[i], d[i+1], d[i+2], d[i+3] = s[j+1], s[j+1], s[j+1], s[j+0] end)
matrix.ag.bgra = matrix.ag.rgba
matrix.ag.argb = eachpixel(function(d, i, s, j) d[i], d[i+1], d[i+2], d[i+3] = s[j+0], s[j+1], s[j+1], s[j+1] end)
matrix.ag.abgr = matrix.ag.argb

matrix.rgb.argb = eachpixel(function(d, i, s, j) d[i], d[i+1], d[i+2], d[i+3] = 0xff, s[j], s[j+1], s[j+2] end)
matrix.bgr.abgr = matrix.rgb.argb
matrix.rgb.rgba = eachpixel(function(d, i, s, j) d[i], d[i+1], d[i+2], d[i+3] = s[j], s[j+1], s[j+2], 0xff end)
matrix.bgr.bgra = matrix.rgb.rgba
matrix.rgb.abgr = eachpixel(function(d, i, s, j) d[i], d[i+1], d[i+2], d[i+3] = 0xff, s[j+2], s[j+1], s[j] end)
matrix.bgr.argb = matrix.rgb.abgr
matrix.rgb.bgra = eachpixel(function(d, i, s, j) d[i], d[i+1], d[i+2], d[i+3] = s[j+2], s[j+1], s[j], 0xff end)
matrix.bgr.rgba = matrix.rgb.bgra

local function rgb2g(r,g,b) return 0.2126 * r + 0.7152 * g + 0.0722 * b end --photometric/digital ITU-R formula

matrix.rgb.g = eachpixel(function(d, i, s, j) d[i] = rgb2g(s[j+0], s[j+1], s[j+2]) end)
matrix.bgr.g = eachpixel(function(d, i, s, j) d[i] = rgb2g(s[j+2], s[j+1], s[j+0]) end)

matrix.rgba.rgb = eachpixel(function(d, i, s, j) d[i], d[i+1], d[i+2] = s[j+0], s[j+1], s[j+2] end)
matrix.bgra.bgr = matrix.rgba.rgb
matrix.argb.rgb = eachpixel(function(d, i, s, j) d[i], d[i+1], d[i+2] = s[j+1], s[j+2], s[j+3] end)
matrix.abgr.bgr = matrix.argb.rgb
matrix.rgba.bgr = eachpixel(function(d, i, s, j) d[i], d[i+1], d[i+2] = s[j+2], s[j+1], s[j+0] end)
matrix.bgra.rgb = matrix.rgba.bgr
matrix.argb.bgr = eachpixel(function(d, i, s, j) d[i], d[i+1], d[i+2] = s[j+3], s[j+2], s[j+1] end)
matrix.abgr.rgb = matrix.argb.bgr

matrix.rgba.ga = eachpixel(function(d, i, s, j) d[i+0], d[i+1] = rgb2g(s[j+0], s[j+1], s[j+2]), s[j+3] end)
matrix.rgba.ag = eachpixel(function(d, i, s, j) d[i+1], d[i+0] = rgb2g(s[j+0], s[j+1], s[j+2]), s[j+3] end)
matrix.bgra.ga = eachpixel(function(d, i, s, j) d[i+0], d[i+1] = rgb2g(s[j+2], s[j+1], s[j+0]), s[j+3] end)
matrix.bgra.ag = eachpixel(function(d, i, s, j) d[i+1], d[i+0] = rgb2g(s[j+2], s[j+1], s[j+0]), s[j+3] end)
matrix.argb.ga = eachpixel(function(d, i, s, j) d[i+0], d[i+1] = rgb2g(s[j+1], s[j+2], s[j+3]), s[j+0] end)
matrix.argb.ag = eachpixel(function(d, i, s, j) d[i+1], d[i+0] = rgb2g(s[j+1], s[j+2], s[j+3]), s[j+0] end)
matrix.abgr.ga = eachpixel(function(d, i, s, j) d[i+0], d[i+1] = rgb2g(s[j+3], s[j+2], s[j+1]), s[j+0] end)
matrix.abgr.ag = eachpixel(function(d, i, s, j) d[i+1], d[i+0] = rgb2g(s[j+3], s[j+2], s[j+1]), s[j+0] end)

matrix.rgba.g = eachpixel(function(d, i, s, j) d[i] = rgb2g(s[j+0], s[j+1], s[j+2]) end)
matrix.bgra.g = eachpixel(function(d, i, s, j) d[i] = rgb2g(s[j+2], s[j+1], s[j+0]) end)
matrix.argb.g = eachpixel(function(d, i, s, j) d[i] = rgb2g(s[j+1], s[j+2], s[j+3]) end)
matrix.abgr.g = eachpixel(function(d, i, s, j) d[i] = rgb2g(s[j+3], s[j+2], s[j+1]) end)

matrix.rgb.ga = eachpixel(function(d, i, s, j) d[i+0], d[i+1] = rgb2g(s[j+0], s[j+1], s[j+2]), 0xff end)
matrix.rgb.ag = eachpixel(function(d, i, s, j) d[i+1], d[i+0] = rgb2g(s[j+0], s[j+1], s[j+2]), 0xff end)
matrix.bgr.ga = eachpixel(function(d, i, s, j) d[i+0], d[i+1] = rgb2g(s[j+2], s[j+1], s[j+0]), 0xff end)
matrix.bgr.ag = eachpixel(function(d, i, s, j) d[i+1], d[i+0] = rgb2g(s[j+2], s[j+1], s[j+0]), 0xff end)

matrix.ga.g = eachpixel(function(d, i, s, j) d[i] = s[j+0] end)
matrix.ag.g = eachpixel(function(d, i, s, j) d[i] = s[j+1] end)

matrix.ga.rgb = eachpixel(function(d, i, s, j) d[i], d[i+1], d[i+2] = s[j+0], s[j+0], s[j+0] end)
matrix.ga.bgr = matrix.ga.rgb
matrix.ag.rgb = eachpixel(function(d, i, s, j) d[i], d[i+1], d[i+2] = s[j+1], s[j+1], s[j+1] end)
matrix.ag.bgr = matrix.ag.rgb

local function inv_cmyk2rgb(c, m, y, k) return c * k / 255, m * k / 255, y * k / 255 end --from webkit

matrix.cmyk.rgb  = eachpixel(function(d, i, s, j) d[i+0], d[i+1], d[i+2] = inv_cmyk2rgb(s[j], s[j+1], s[j+2], s[j+3]) end)
matrix.cmyk.bgr  = eachpixel(function(d, i, s, j) d[i+2], d[i+1], d[i+0] = inv_cmyk2rgb(s[j], s[j+1], s[j+2], s[j+3]) end)
matrix.cmyk.rgba = eachpixel(function(d, i, s, j) d[i+0], d[i+1], d[i+2] = inv_cmyk2rgb(s[j], s[j+1], s[j+2], s[j+3]); d[i+3] = 0xff end)
matrix.cmyk.bgra = eachpixel(function(d, i, s, j) d[i+2], d[i+1], d[i+0] = inv_cmyk2rgb(s[j], s[j+1], s[j+2], s[j+3]); d[i+3] = 0xff end)
matrix.cmyk.argb = eachpixel(function(d, i, s, j) d[i+1], d[i+2], d[i+3] = inv_cmyk2rgb(s[j], s[j+1], s[j+2], s[j+3]); d[i+0] = 0xff end)
matrix.cmyk.abgr = eachpixel(function(d, i, s, j) d[i+3], d[i+2], d[i+1] = inv_cmyk2rgb(s[j], s[j+1], s[j+2], s[j+3]); d[i+0] = 0xff end)
matrix.cmyk.g = eachpixel(function(d, i, s, j) d[i] = rgb2g(inv_cmyk2rgb(s[j], s[j+1], s[j+2], s[j+3])) end)
matrix.cmyk.ga = eachpixel(function(d, i, s, j) d[i+0], d[i+1] = rgb2g(inv_cmyk2rgb(s[j], s[j+1], s[j+2], s[j+3])), 0xff end)
matrix.cmyk.ag = eachpixel(function(d, i, s, j) d[i+1], d[i+0] = rgb2g(inv_cmyk2rgb(s[j], s[j+1], s[j+2], s[j+3])), 0xff end)

--frontend

local ffi = require'ffi'
local bit = require'bit'

local function pad_stride(stride) --increase stride to the next number divisible by 4
	return bit.band(stride + 3, bit.bnot(3))
end

local function supported(src, dst)
	return src == dst or (matrix[src] and matrix[src][dst] and true or false)
end

local function split_range(x1, x2, n) --split a numeric range in N equal ranges
	--
end

local function split_work(src, dst, operation, threads) --split operation to multiple lanes
	threads = math.min(threads, src.h)
	local lanes = require'lanes'.configure()

	local src_data = src.data
	local dst_data = dst.data
	src.data = tonumber(ffi.cast('uint32_t', ffi.cast('void*', src.data))) --pointers can only reach lanes as numbers
	dst.data = tonumber(ffi.cast('uint32_t', ffi.cast('void*', dst.data)))

	local linda = lanes.linda()
	local op_thread = lanes.gen(operation)
	local tt = {}
	local next_range = split_range(0, src.h-1, threads)
	for i=1,threads do
		local h1, h2 = next_range()
		tt[#tt+1] = op_thread(src, dst, h1, h2) --each thread will work on a separate section of the buffer
	end
	for _,thread in ipairs(tt) do --TODO: wait for threads
		local _ = thread[1]
	end

	src.data = src_data
	dst.data = dst_data
end

local MIN_SIZE_PER_THREAD = 1024 * 1024 --1MB/thread to make it worth to create one

local function convert(src, fmt, opt)
	local dst = {}
	for k,v in pairs(src) do dst[k] = v end --all image info gets copied
	dst.pixel = fmt.pixel
	dst.stride = fmt.stride
	dst.orientation = fmt.orientation

	--see if there's anything to convert
	if src.pixel == fmt.pixel
		and src.stride == fmt.stride
		and src.orientation == fmt.orientation
	then
		return dst
	end

	--check consistency of the input
	--NOTE: we support unknown pixel formats as long as #pixel == pixel size in bytes
	assert(src.size == src.h * src.stride)
	assert(src.stride >= src.w * #src.pixel)
	assert(fmt.stride >= src.w * #fmt.pixel)
	assert(src.orientation == 'top_down' or src.orientation == 'bottom_up')
	assert(fmt.orientation == 'top_down' or fmt.orientation == 'bottom_up')
	assert(supported(src.pixel, fmt.pixel))

	--see if there's a dest. buffer, or we can overwrite src. or we need to alloc. one
	if opt.data then
		assert(opt.size >= src.h * fmt.stride)
		dst.size = opt.size
		dst.data = opt.data
	elseif (opt and opt.force_copy)
		or src.stride ~= fmt.stride --diff. buffer size
		or src.orientation ~= fmt.orientation --needs flippin'
		or #fmt.pixel > #src.pixel --bigger pixel, even if same row size
	then
		dst.size = src.h * fmt.stride
		dst.data = ffi.new('uint8_t[?]', dst.size)
	end

	--see if we need a pixel conversion or just flipping and/or changing stride
	local operation = src.pixel == fmt.pixel and copy_rows or matrix[src.pixel][fmt.pixel]

	--[[
	if opt and opt.threads and opt.threads > 1 then
		local threads = math.min(opt.threads, math.floor(src.size / MIN_SIZE_PER_THREAD))
		and src.h > opt.threads
		and src.size > MIN_SIZE_PER_THREAD * 2 --it's worth the overhead of creating threads
	then
		split_work(src, dst, operation, math.min(opt.threads - 1))
	else
	]]
	operation(src, dst, 0, src.h)
	--end
	return dst
end

local preferred_formats = {
	g = {'ga', 'ag', 'rgb', 'bgr', 'rgba', 'bgra', 'argb', 'abgr'},
	ga = {'ag', 'rgba', 'bgra', 'argb', 'abgr', 'rgb', 'bgr', 'g'},
	ag = {'ga', 'rgba', 'bgra', 'argb', 'abgr', 'rgb', 'bgr', 'g'},
	rgb = {'bgr', 'rgba', 'bgra', 'argb', 'abgr', 'g', 'ga', 'ag'},
	bgr = {'rgb', 'rgba', 'bgra', 'argb', 'abgr', 'g', 'ga', 'ag'},
	rgba = {'bgra', 'argb', 'abgr', 'rgb', 'bgr', 'ga', 'ag', 'g'},
	bgra = {'rgba', 'argb', 'abgr', 'rgb', 'bgr', 'ga', 'ag', 'g'},
	argb = {'rgba', 'bgra', 'abgr', 'rgb', 'bgr', 'ga', 'ag', 'g'},
	abgr = {'rgba', 'bgra', 'argb', 'rgb', 'bgr', 'ga', 'ag', 'g'},
	cmyk = {'rgb', 'bgr', 'rgba', 'bgra', 'argb', 'abgr', 'g', 'ga', 'ag'},
}

local function best_format(src, accept)
	local fmt = {}
	assert(src.stride)
	fmt.stride = accept and accept.padded and pad_stride(src.stride) or src.stride
	assert(src.orientation == 'top_down' or src.orientation == 'bottom_up')
	fmt.orientation =
		(not accept or accept.top_down == nil and accept.bottom_up == nil) and src.orientation --no preference, keep it
		or accept[src.orientation] and src.orientation --same as source, keep it
		or accept.top_down and 'top_down'
		or accept.bottom_up and 'bottom_up'
	assert(src.pixel)
	if not accept or accept[src.pixel] then --source pixel format accepted, keep it, even if unknown
		fmt.pixel = src.pixel
		return fmt
	elseif preferred_formats[src.pixel] then --known source pixel format, find it best destination format
		for _,pixel in ipairs(preferred_formats[src.pixel]) do
			if accept[pixel] and matrix[src.pixel][pixel] then --we must have an implementation for it
				fmt.pixel = pixel
				fmt.stride = src.w * #pixel
				if accept.padded then fmt.stride = pad_stride(fmt.stride) end
				return fmt
			end
		end
	end
end

local function convert_best(src, accept, opt)
	local fmt = best_format(src, accept)

	if not fmt then
		local t = {}; for k in pairs(accept) do t[#t+1] = k end
		error(string.format('cannot convert from (%s, %s) to (%s)',
									src.pixel, src.orientation, table.concat(t, ', ')))
	end

	return convert(src, fmt, opt)
end

if not ... then require'bmpconv_test' end

return {
	pad_stride = pad_stride,
	convert = convert,
	best_format = best_format,
	convert_best = convert_best,
	supported = supported,
	preferred_formats = preferred_formats,
	matrix = matrix,
}

