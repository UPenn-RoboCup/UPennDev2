--libpng binding for libpng 1.5.6+
local ffi = require'ffi'
local bit = require'bit'
local glue = require'glue'
local bmpconv = require'bmpconv'
require'libpng_h'
require'stdio_h'
local C = ffi.load'png'

local PNG_LIBPNG_VER_STRING = '1.5.10'

local function cdata_reader(data, size)
	data = ffi.cast('uint8_t*', data) --ensure byte type
	return function(_, buf, sz)
		if sz < 1 or size < 1 then error'reading past eof' end
		sz = math.min(size, tonumber(sz))
		ffi.copy(buf, data, sz)
		data = data + sz
		size = size - sz
		return sz
	end
end

local function string_reader(s)
	return cdata_reader(ffi.cast('const uint8_t*', s), #s) --const ensures no copying
end

local function cdata_source_reader(read)
	error'NYI'
end

local function string_source_reader(read)
	error'NYI'
end

local pixel_formats = {
	[C.PNG_COLOR_TYPE_GRAY] = 'g',
	[C.PNG_COLOR_TYPE_RGB] = 'rgb',
	[C.PNG_COLOR_TYPE_RGB_ALPHA] = 'rgba',
	[C.PNG_COLOR_TYPE_GRAY_ALPHA] = 'ga',
}

function save(path, width, height, prgb_data)
  return glue.fcall(function(finally)
    local png_ptr = assert(C.png_create_write_struct(PNG_LIBPNG_VER_STRING, nil, nil, nil))
    local info_ptr = assert(C.png_create_info_struct(png_ptr))
    finally(function()
      local png_ptr = ffi.new('png_structp[1]', png_ptr)
      local info_ptr = ffi.new('png_infop[1]', info_ptr)
      C.png_destroy_write_struct(png_ptr, nil)
    end)

		--setup error handling
		local warnings = {}
		local warning_cb = ffi.cast('png_error_ptr', function(png_ptr, err)
			warnings[#warnings+1] = ffi.string(err)
		end)
		local error_cb = ffi.cast('png_error_ptr', function(png_ptr, err)
			error(string.format('libpng error: %s', ffi.string(err)))
		end)
		finally(function()
			C.png_set_error_fn(png_ptr, nil, nil, nil)
			error_cb:free()
			warning_cb:free()
		end)
		C.png_set_error_fn(png_ptr, nil, error_cb, warning_cb)

		local f = ffi.C.fopen(path, 'wb')
		glue.assert(f ~= nil, 'could not open file %s', path)
		finally(function()
			C.png_init_io(png_ptr, nil)
			ffi.C.fclose(f)
		end)
		C.png_init_io(png_ptr, f)

    C.png_set_IHDR(png_ptr, info_ptr, width, height, 8, 
                C.PNG_COLOR_TYPE_RGB, C.PNG_INTERLACE_NONE, 
                C.PNG_COMPRESSION_TYPE_DEFAULT, C.PNG_FILTER_TYPE_DEFAULT)

--    local text_ptr = ffi.new('png_text[3]', nil)
--    text_ptr[0].

    C.png_write_info(png_ptr, info_ptr)

		local rows_ptr = ffi.new('uint8_t*[?]', height)
    for l = 0, height - 1 do
      rows_ptr[l] = prgb_data + l * width * 3
    end

    C.png_write_image(png_ptr, rows_ptr)
    C.png_write_end(png_ptr, info_ptr)

--    C.png_destroy_write_struct(png_ptr, info_ptr)

  end)
end

local function load(src, opt)
	return glue.fcall(function(finally)
		opt = opt or {}

		--create the state objects
		local png_ptr = assert(C.png_create_read_struct(PNG_LIBPNG_VER_STRING, nil, nil, nil))
		local info_ptr = assert(C.png_create_info_struct(png_ptr))
		finally(function()
			local png_ptr = ffi.new('png_structp[1]', png_ptr)
			local info_ptr = ffi.new('png_infop[1]', info_ptr)
			C.png_destroy_read_struct(png_ptr, info_ptr, nil)
		end)

		--setup error handling
		local warnings = {}
		local warning_cb = ffi.cast('png_error_ptr', function(png_ptr, err)
			warnings[#warnings+1] = ffi.string(err)
		end)
		local error_cb = ffi.cast('png_error_ptr', function(png_ptr, err)
			error(string.format('libpng error: %s', ffi.string(err)))
		end)
		finally(function()
			C.png_set_error_fn(png_ptr, nil, nil, nil)
			error_cb:free()
			warning_cb:free()
		end)
		C.png_set_error_fn(png_ptr, nil, error_cb, warning_cb)

		--setup input source
		if src.string or src.cdata or src.string_source or src.cdata_source then
			local reader =
				src.string and string_reader(src.string)
				or src.cdata and cdata_reader(src.cdata, src.size)
				or src.cdata_source and cdata_source_reader(src.cdata_source)
				or src.string_source and string_source_reader(src.string_source)
			local read_cb = ffi.cast('png_rw_ptr', reader)
			finally(function()
				C.png_set_read_fn(png_ptr, nil, nil)
				read_cb:free()
			end)
			C.png_set_read_fn(png_ptr, nil, read_cb)
		elseif src.path then
			local f = ffi.C.fopen(src.path, 'rb')
			glue.assert(f ~= nil, 'could not open file %s', src.path)
			finally(function()
				C.png_init_io(png_ptr, nil)
				ffi.C.fclose(f)
			end)
			C.png_init_io(png_ptr, f)
		elseif src.stream then
			C.png_init_io(png_ptr, src.stream)
		else
			error'invalid data source (string, cdata/size, path, stream, cdata_source, string_source accepted)'
		end

		--read header
		C.png_read_info(png_ptr, info_ptr)

		--setup mandatory conversion options
		C.png_set_expand(png_ptr) --1,2,4bpp -> 8bpp, palette -> 8bpp, tRNS -> alpha
		C.png_set_scale_16(png_ptr) --16bpp -> 8bpp; since 1.5.4+
		C.png_set_interlace_handling(png_ptr) --deinterlace
		C.png_read_update_info(png_ptr, info_ptr)

		--get dimensions and pixel format information
		local w = C.png_get_image_width(png_ptr, info_ptr)
		local h = C.png_get_image_height(png_ptr, info_ptr)
		local color_type = C.png_get_color_type(png_ptr, info_ptr)
		color_type = bit.band(color_type, bit.bnot(C.PNG_COLOR_MASK_PALETTE))
		local paletted = bit.band(color_type, C.PNG_COLOR_MASK_PALETTE) == C.PNG_COLOR_MASK_PALETTE
		local pixel_format = assert(pixel_formats[color_type])

		local gamma = opt.gamma or 2.2
		local function set_alpha(png_ptr)
			C.png_set_alpha_mode(png_ptr, C.PNG_ALPHA_OPTIMIZED, gamma) --> premultiply alpha
		end

		--request more conversions depending on pixel_format and the accept table
		local accept = opt.accept or {}

		local function strip_alpha(png_ptr)
			local my_background = ffi.new('png_color_16', 0, 0xff, 0xff, 0xff, 0xff)
			local image_background = ffi.new'png_color_16'
			local image_background_p = ffi.new('png_color_16p[1]', image_background)
			if C.png_get_bKGD(png_ptr, info_ptr, image_background_p) then
				C.png_set_background(png_ptr, image_background, C.PNG_BACKGROUND_GAMMA_FILE, 1, 1.0)
			else
				C.png_set_background(png_ptr, my_background, PNG_BACKGROUND_GAMMA_SCREEN, 0, 1.0)
			end
		end

		local dest_pixel_format = pixel_format
		if pixel_format == 'g' then
			if accept.g then
				--we're good
			elseif accept.ga then
				C.png_set_add_alpha(png_ptr, 0xff, C.PNG_FILLER_AFTER)
				dest_pixel_format = 'ga'
			elseif accept.ag then
				C.png_set_add_alpha(png_ptr, 0xff, C.PNG_FILLER_BEFORE)
				dest_pixel_format = 'ag'
			elseif accept.rgb then
				C.png_set_gray_to_rgb(png_ptr)
				dest_pixel_format = 'rgb'
			elseif accept.bgr then
				C.png_set_gray_to_rgb(png_ptr)
				C.png_set_bgr(png_ptr)
				dest_pixel_format = 'bgr'
			elseif accept.rgba then
				C.png_set_gray_to_rgb(png_ptr)
				C.png_set_add_alpha(png_ptr, 0xff, C.PNG_FILLER_AFTER)
				dest_pixel_format = 'rgba'
			elseif accept.argb then
				C.png_set_gray_to_rgb(png_ptr)
				C.png_set_add_alpha(png_ptr, 0xff, C.PNG_FILLER_BEFORE)
				dest_pixel_format = 'argb'
			elseif accept.bgra then
				C.png_set_gray_to_rgb(png_ptr)
				C.png_set_bgr(png_ptr)
				C.png_set_add_alpha(png_ptr, 0xff, C.PNG_FILLER_AFTER)
				dest_pixel_format = 'bgra'
			elseif accept.abgr then
				C.png_set_gray_to_rgb(png_ptr)
				C.png_set_bgr(png_ptr)
				C.png_set_add_alpha(png_ptr, 0xff, C.PNG_FILLER_BEFORE)
				dest_pixel_format = 'abgr'
			end
		elseif pixel_format == 'ga' then
			if accept.ga then
				set_alpha(png_ptr)
			elseif accept.ag then
				C.png_set_swap_alpha(png_ptr)
				set_alpha(png_ptr)
				dest_pixel_format = 'ag'
			elseif accept.rgba then
				C.png_set_gray_to_rgb(png_ptr)
				set_alpha(png_ptr)
				dest_pixel_format = 'rgba'
			elseif accept.argb then
				C.png_set_gray_to_rgb(png_ptr)
				C.png_set_swap_alpha(png_ptr)
				set_alpha(png_ptr)
				dest_pixel_format = 'argb'
			elseif accept.bgra then
				C.png_set_gray_to_rgb(png_ptr)
				C.png_set_bgr(png_ptr)
				set_alpha(png_ptr)
				dest_pixel_format = 'bgra'
			elseif accept.abgr then
				C.png_set_gray_to_rgb(png_ptr)
				C.png_set_bgr(png_ptr)
				C.png_set_swap_alpha(png_ptr)
				set_alpha(png_ptr)
				dest_pixel_format = 'abgr'
			elseif accept.g then
				strip_alpha(png_ptr)
				dest_pixel_format = 'g'
			elseif accept.rgb then
				C.png_set_gray_to_rgb(png_ptr)
				strip_alpha(png_ptr)
				dest_pixel_format = 'rgb'
			elseif accept.bgr then
				C.png_set_gray_to_rgb(png_ptr)
				C.png_set_bgr(png_ptr)
				strip_alpha(png_ptr)
				dest_pixel_format = 'bgr'
			else
				set_alpha(png_ptr)
			end
		elseif pixel_format == 'rgb' then
			if accept.rgb then
				--we're good
			elseif accept.bgr then
				C.png_set_bgr(png_ptr)
				dest_pixel_format = 'bgr'
			elseif accept.rgba then
				C.png_set_add_alpha(png_ptr, 0xff, C.PNG_FILLER_AFTER)
				dest_pixel_format = 'rgba'
			elseif accept.argb then
				C.png_set_add_alpha(png_ptr, 0xff, C.PNG_FILLER_BEFORE)
				dest_pixel_format = 'argb'
			elseif accept.bgra then
				C.png_set_bgr(png_ptr)
				C.png_set_add_alpha(png_ptr, 0xff, C.PNG_FILLER_AFTER)
				dest_pixel_format = 'bgra'
			elseif accept.abgr then
				C.png_set_bgr(png_ptr)
				C.png_set_add_alpha(png_ptr, 0xff, C.PNG_FILLER_BEFORE)
				dest_pixel_format = 'abgr'
			elseif accept.g then
				C.png_set_rgb_to_gray_fixed(png_ptr, 1, -1, -1)
				dest_pixel_format = 'g'
			elseif accept.ga then
				C.png_set_rgb_to_gray_fixed(png_ptr, 1, -1, -1)
				C.png_set_add_alpha(png_ptr, 0xff, C.PNG_FILLER_AFTER)
				dest_pixel_format = 'ga'
			elseif accept.ag then
				C.png_set_rgb_to_gray_fixed(png_ptr, 1, -1, -1)
				C.png_set_add_alpha(png_ptr, 0xff, C.PNG_FILLER_BEFORE)
				dest_pixel_format = 'ag'
			end
		elseif pixel_format == 'rgba' then
			if accept.rgba then
				set_alpha(png_ptr)
			elseif accept.argb then
				C.png_set_swap_alpha(png_ptr)
				set_alpha(png_ptr)
				dest_pixel_format = 'argb'
			elseif accept.bgra then
				C.png_set_bgr(png_ptr)
				set_alpha(png_ptr)
				dest_pixel_format = 'bgra'
			elseif accept.abgr then
				C.png_set_bgr(png_ptr)
				C.png_set_swap_alpha(png_ptr)
				set_alpha(png_ptr)
				dest_pixel_format = 'abgr'
			elseif accept.rgb then
				strip_alpha(png_ptr)
				dest_pixel_format = 'rgb'
			elseif accept.bgr then
				C.png_set_bgr(png_ptr)
				strip_alpha(png_ptr)
				dest_pixel_format = 'bgr'
			elseif accept.ga then
				C.png_set_rgb_to_gray_fixed(png_ptr, 1, -1, -1)
				dest_pixel_format = 'ga'
			elseif accept.ag then
				C.png_set_rgb_to_gray_fixed(png_ptr, 1, -1, -1)
				C.png_set_swap_alpha(png_ptr)
				dest_pixel_format = 'ag'
			elseif accept.g then
				C.png_set_rgb_to_gray_fixed(png_ptr, 1, -1, -1)
				strip_alpha(png_ptr)
				dest_pixel_format = 'g'
			else
				set_alpha(png_ptr)
			end
		else
			assert(false)
		end
		C.png_read_update_info(png_ptr, info_ptr) --calling this twice is libpng 1.5.6+

		--check if conversion options had the desired effect
		assert(C.png_get_bit_depth(png_ptr, info_ptr) == 8)

		local color_type = C.png_get_color_type(png_ptr, info_ptr)
		local actual_pixel_format = assert(pixel_formats[color_type])
		assert(#actual_pixel_format == #dest_pixel_format) --same number of channels

		local channels = C.png_get_channels(png_ptr, info_ptr)
		assert(channels == #actual_pixel_format) --each letter a channel

		local stride = w * channels
		assert(C.png_get_rowbytes(png_ptr, info_ptr) == stride)
		if accept.padded then stride = bmpconv.pad_stride(stride) end

		local bottom_up = accept.bottom_up and not accept.top_down

		--get the data bits
		local size = stride * h
		local data = ffi.new('uint8_t[?]', size)
		local rows_ptr = ffi.new('uint8_t*[?]', h)
		if bottom_up then
			for i=0,h-1 do
				rows_ptr[h-1-i] = data + (stride * i)
			end
		else
			for i=0,h-1 do
				rows_ptr[i] = data + (stride * i)
			end
		end
		C.png_read_image(png_ptr, rows_ptr)
		C.png_read_end(png_ptr, info_ptr)

		local img = {
			data = data,
			size = size,
			pixel = dest_pixel_format,
			stride = stride,
			orientation = bottom_up and 'bottom_up' or 'top_down',
			w = w,
			h = h,
			warnings = warnings,
			file_format = pixel_format,
			paletted = paletted,
		}
		return bmpconv.convert_best(img, opt.accept)
	end)
end

if not ... then require'libpng_test' end

return {
	load = load,
  save = save,
	C = C,
}
