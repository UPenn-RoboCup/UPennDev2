dofile'include.lua'
require'unix'
mp = require'msgpack'
util = require'util'
fname = 'load.log'

local function log()

local fl = io.open(fname,'w')
local ntimes = 5
local urate  = 5e5
for i=1,ntimes do
f = io.popen('ps -C lua -o cmd=Process,%cpu,pmem,rss=kilobytes,psr')
t = unix.time()
header=f:read()
local p = true
local l = {t=t}
while p do
	p = f:read()
	if not p then break end
	-- Get the process
	--local process = p:match('lua (%S*)%.lua')
	local process = p:match('lua (%S*)_wizard%.lua')
	-- Get the numbers
	n=p:gmatch('%d+%.*%d*')
	local pid = {}
	pid.pcpu = n()
	pid.pmem = n()
	pid.kb   = n()
	pid.core = n()
	l[process] = pid
end
local log = mp.pack(l)
fl:write(log)
unix.usleep(urate)
end
fl:close()

end

local function unlog()
local fl = io.open(fname,'r')
local file_str = fl:read('*a')
local unpacker = msgpack.unpacker(file_str)
local tbl = true
while tbl do
  tbl = unpacker:unpack()
  if not tbl then break end
  -- Format
  util.ptable(tbl)
end

end

unlog()
