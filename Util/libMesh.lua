local libMesh = {}
local ffi = require'ffi'
-- Compression utilities
local p_compress = require'png'.compress
local j_compress = require'jpeg'.compressor'gray'
local min = require'math'.min
local max = require'math'.max
local floor = require'math'.floor

-- Range compression method to bring float into byte
local function dynamic_range(self, dynrange)
	self.metadata.dr = dynrange or self.metadata.dr
	local near, far = unpack(self.metadata.dr)
	--print('near, far', near, far)
	local factor = 255 / (far-near)
	for i=0, self.n_el-1 do
		self.byte[i] = max(0, min(factor*(self.raw[i] - near), 255))
	end
end

local function get_raw_string(self)
	return ffi.string(self.raw, ffi.sizeof(self.raw))
end
local function get_byte_string(self)
	return ffi.string(self.byte, ffi.sizeof(self.byte))
end
local function get_jpeg_string(self)
	return ffi.string(self.byte, ffi.sizeof(self.byte))
end
local function get_png_string(self)
	local h, w = unpack(self.metadata.dim)
	return p_compress(tonumber(ffi.cast('intptr_t', self.byte)), w, h, 1)
end
local function get_png_string2(self)
	local h, w = unpack(self.metadata.dim)
	return p_compress(ffi.string(self.byte, ffi.sizeof(self.byte)), w, h, 1)
end

local function save(self, fname_raw, fname_byte)
	--print('dim', unpack(self.metadata.dim))
	local f = io.open(fname_raw, 'w')
	f:write(self:get_raw_string())
	f:close()
	f = io.open(fname_byte, 'w')
	f:write(self:get_byte_string())
	f:close()
end

-- Convert an actuator angle to a scanline of the mesh image
-- Need: Sensor position in radians
local function get_scanline(self, rad)
	-- Find the scanline
	local ratio = (rad - self.metadata.sfov[1]) / (self.metadata.sfov[2] - self.metadata.sfov[1])
	-- Keep within range
	ratio = max(0, min(ratio, 1))
	local scanline = ratio * self.metadata.dim[1]
	-- Floor for zero-based indexing
	return floor(scanline)
end

-- Add a scan to the mesh
local function add_scan(self, lidar, scan)
	-- Get the scanline
	local scanline = get_scanline(self, lidar.angle)
	local scanline_offset_fl = scanline * self.metadata.dim[2]
	local scan_sz_bytes = ffi.sizeof'float' * self.metadata.dim[2]
	-- Copy the scan
	ffi.copy(self.raw + scanline_offset_fl, ffi.cast('float*', scan) + self.scan_offset, scan_sz_bytes)

	-- Save the metadata
	self.metadata.a[scanline + 1] = lidar.angle
	self.metadata.tfL6[scanline + 1] = lidar.tfL6
	self.metadata.tfG6[scanline + 1] = lidar.tfG6
end

-- id: ID of the mesh
-- p: properties
function libMesh.new(id, meta)
	local n, res = meta.n_lidar_returns, meta.lidar_resolution
	local lidar_fov = n * res
	-- Check that we can access enough FOV
	local rfov0, rfov1 = unpack(meta.rfov)
	assert(rfov1 >= rfov0, 'Ensure the R Field of View is correct')
	local mesh_lidar_fov = rfov1 - rfov0
	assert(lidar_fov >= mesh_lidar_fov, 'Ensure the sensor provides enough Field of View')
	-- Find the offset for copying lidar readings into the mesh
	-- if fov is from -fov/2 to fov/2 degrees, then offset_idx is zero
	-- if fov is from 0 to fov/2 degrees, then offset_idx is sensor_width/2
	local fov_offset = rfov0 / res + (n / 2)
	-- Round the offset (0 based offset)
	local offset_idx = floor(fov_offset + 0.5)
	-- Round to get the number of returns for each scanline
	local n_returns = floor(mesh_lidar_fov / res + 0.5)
	-- Check the scanline fov
	local sfov0, sfov1 = unpack(meta.sfov)
	assert(sfov1 >= sfov0, 'Ensure the S Field of View is correct')
	local n_scanlines = meta.n_scanlines

	-- Setup our container
	local o = {}
	-- Metadata when sending out
	o.metadata = {
		id = id,
		t = -math.huge,
		c = 'raw', --'jpeg', 'png' compression types
		dr = {0, 30}, -- Dynamic range
		rfov = {rfov0, rfov1},
		sfov = {sfov0, sfov1},
		dim = {n_scanlines, n_returns},
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
	o.scan_offset = offset_idx
	-- Functions
	o.add_scan = add_scan
	o.dynamic_range = dynamic_range
	o.get_raw_string = get_raw_string
	o.get_jpeg_string = get_jpeg_string
	o.get_png_string = get_png_string
	o.get_png_string2 = get_png_string2
	o.get_byte_string = get_byte_string
	o.save = save

	return o
end

return libMesh
