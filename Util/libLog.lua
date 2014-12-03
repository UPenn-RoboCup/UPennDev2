local libLog = {}
local LOG_DIR = '/tmp'
local carray
if ffi then
	ffi.cdef [[
	typedef struct __IO_FILE FILE;
	size_t fwrite
	(const void *restrict ptr, size_t size, size_t nitems, FILE *restrict stream);
	size_t fread
	(void *restrict ptr, size_t size, size_t nitems, FILE *restrict stream);
	]]
else
	-- TODO: Maybe use a pcall here in case carray not found
	carray = require'carray'
end
local torch = require'torch'
local mpack = require'msgpack.MessagePack'.pack
-- Need the C version for unpacker
local munpacker = require'msgpack'.unpacker

-- TODO: __gc should call stop
local function stop(self)
	-- Close the files
	self.f_meta:close()
	if self.f_raw then self.f_raw:close() end
end

-- User should pass :data() of a torch object
-- For proper recording
local function record(self, meta, raw, n_raw)
	-- Record the metadata
	local mtype, m_ok = type(meta), false
	if mtype=='string' then
		m_ok = self.f_meta:write(meta)
	elseif mtype then
		m_ok = self.f_meta:write(mpack(meta))
	end
	-- Record the raw
	local rtype, r_ok = type(raw), false
	if rtype=='userdata' or rtype=='cdata' then
		-- If no FFI, then cannot record userdata
		-- If no number of raw data, then cannot record
		-- TODO: Use carray as FFI fallback
		if not n_raw then return end
		if C then
			local n_written = C.fwrite(raw, 1, n_raw, self.f_raw)
			--print('wrote',n_written)
			r_ok = n_written==n_raw
		else
			r_ok = self.f_raw:write(tostring(carray.byte(raw ,n_raw)))
		end
	elseif rtype=='string' then
		r_ok = self.f_raw:write(raw)
	end
	-- Return the status of the writes
  self.n = self.n + 1
	return m_ok, r_ok
end


-- Factory
function libLog.new(prefix, has_raw)
	-- Set up log file handles
  local filetime = os.date('%m.%d.%Y.%H.%M.%S')
  local meta_filename = string.format('%s/%s_m_%s.log',LOG_DIR,prefix,filetime)
	local raw_filename  = string.format('%s/%s_r_%s.log',LOG_DIR,prefix,filetime)
	local f_meta = io.open(meta_filename,'w')
	local f_raw, f_raw_c
	if has_raw then f_raw = io.open(raw_filename,'w') end
	-- Set up the object
	local t = {}
	t.f_raw = f_raw
	t.f_meta = f_meta
	t.record = record
	t.stop = stop
  t.n = 0
	return t
end

local function unroll_meta(self)
	-- Read the metadata
	local f_m = assert(io.open(self.m_name,'r'))
	-- Must use an unpacker...
	local metadata = {}
	local u = munpacker(2048)
	local buf, nbuf = f_m:read(512),0
	while buf do
		nbuf = nbuf + #buf
		local res,left = u:feed(buf)
		local tbl = u:pull()
		while tbl do
			metadata[#metadata+1] = tbl
			tbl = u:pull()
		end
		buf = f_m:read(left)
	end
	f_m:close()
  self.metadata = metadata
	return metadata
end

local function log_iter(self)
	local metadata, buf_t = self.metadata
	local f_r = io.open(self.r_name,'r')
	local i, n = 0, #metadata
	if C then buf_t = torch.ByteTensor() end
	local function iter(param, state)
		i = i + 1
		if i>n then
			if f_r then f_r:close() end
			return nil
		end
		--if not param then return end
		local m = metadata[i]
		-- Metadata only
		if not f_r then return i, m end
		if C then
			buf_t:resize(m.rsz)
			local n_read = C.fread(buf_t:data(),1,m.rsz,f_r)
			return i, m, buf_t
		else
			local data = f_r:read(m.rsz)
			return i, m, data
		end
	end
	return iter
end

function libLog.open(dir,date,prefix)
	local t = {}
	t.m_name = dir..'/'..prefix..'_m_'..date..'.log'
	t.r_name = dir..'/'..prefix..'_r_'..date..'.log'
	t.unroll_meta = unroll_meta
	t.log_iter = log_iter
	return t
end

-- Fill the metatable

return libLog
