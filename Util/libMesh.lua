local libMesh = {}
local ffi = require'ffi'
-- Compression utilities
local p_compress = require'png'.compress
local j_compress = require'jpeg'.compressor'gray'
local min = require'math'.min
local max = require'math'.max
local floor = require'math'.floor

-- Range compression method to bring float into byte
local function dynamic_range(self, )
	local near, far = unpack(self.metadata.dr)
	for i=0, o.n_el-1 do
		o.byte[ii] = min(max((o.raw[ii] - near)/(far-near), 0), 255)
	end
end

local function get_raw_string(self)
	return ffi.string(self.raw, ffi.sizeof(self.raw))
end
local function get_jpeg_string(self)
	return ffi.string(self.raw, ffi.sizeof(self.raw))
end
local function get_png_string(self)
	local w, h = unpack(self.metadata.dim)
	return p_compress(tonumber(ffi.cast('intptr_t', self.byte)), w, h, 1)
end
local function get_png_string2(self)
	local w, h = unpack(self.metadata.dim)
	return p_compress(ffi.string(self.byte, ffi.sizeof(self.byte)), w, h, 1)
end

-- Convert an actuator angle to a scanline of the mesh image
-- Need: Sensor position in radians
local function get_scanline(self, rad)
	-- Find the scanline
	local ratio = (rad - self.metadata.sfov[1]) / (self.metadata.sfov[2] - self.metadata.sfov[1])
	-- Keep within range
	ratio = max(0, min(ratio, 1))
	local scanline = ratio * self.metadata.n_scanlines
	-- Floor for zero-based indexing
	return floor(scanline)
end

-- Add a scan to the mesh
local function add_scan(self, meta, scan)
	-- Get the scanline
	local scanline = get_scanline(self, meta.angle)
	local scanline_offset = scanline * self.scanline_sz
	-- Copy the scan
	ffi.copy(self.raw + scanline_offset, ffi.cast('float*', scan) + self.scan_offset, self.scan_sz)
	-- Save the metadata
	self.metadata.a[scanline + 1] = meta.angle
	self.metadata.tfL6[scanline + 1] = meta.tfL6
	self.metadata.tfG6[scanline + 1] = meta.tfG6
end

-- id: ID of the mesh
-- p: properties
function libMesh.new(id, p)
	local n, res = meta.n, meta.res
	local fov = n * res
	-- Check that we can access enough FOV
	local min_view, max_view = unpack(ranges_fov)
	assert(fov > max_view-min_view, 'Not enough FOV available')
	-- Find the offset for copying lidar readings into the mesh
	-- if fov is from -fov/2 to fov/2 degrees, then offset_idx is zero
	-- if fov is from 0 to fov/2 degrees, then offset_idx is sensor_width/2
	local fov_offset = min_view / res + n / 2
	-- Round the offset (0 based offset)
	local offset_idx = floor(fov_offset + 0.5)
	-- Round to get the number of returns for each scanline
	local n_returns = floor((max_view - min_view) / res + 0.5)
	local n_scanlines = floor(t_sweep / t_scan + 0.5)

	-- Setup our container
	local o = {}
	-- Metadata when sending out
	o.metadata = {
		id = id,
		t = -math.huge,
		c = 'raw', --'jpeg', 'png' compression types
		dr = {0, 30}, -- Dynamic range
		rfov = ranges_fov,
		sfov = {-mag_sweep / 2, mag_sweep / 2},
		dim = {n_scanlines, n_returns}
		tfL6 = {},
		tfG6 = {},
		a = {}
	}
	-- Initial population for the metadata
	local zero_tf = {0,0,0, 0,0,0}
	for i=1,n_scanlines do
		o.metadata.a[i] = 0
		o.metadata.tfL6[i] = zero_tf
		o.metadata.tfG6[i] = zero_tf
	end
	-- Internal Data
	local n_el = n_scanlines * n_returns
	o.n_el = n_el
	o.raw = ffi.new("float[?]", n_el)
	o.byte = ffi.new("uint8_t[?]", n_el)
	o.scan_sz = n_returns * ffi.sizeof'float'
	o.scan_offset = offset_idx
	-- Functions
	o.dynamic_range = dynamic_range
	o.get_raw_string = get_raw_string
	o.get_jpeg_string = get_jpeg_string
	o.get_png_string = get_png_string

	return o
end

return libMesh
