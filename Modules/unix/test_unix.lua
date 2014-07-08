dofile'../../include.lua'
local util = require'util'
local unix = require'unix'
local unix2 = require'unix.ffi'

print("\n\nORIGI")
util.ptable(unix)
print("\n\nNEW")
util.ptable(unix2)

print("\n\nTIME TEST")
local t0 = unix.time()
local t1 = unix2.time()
print(t0, t1)

--[[
print("\nuSLEEP TEST 1")
unix.usleep(5e5)
print("\nuSLEEP TEST 2")
unix2.usleep(5e5)

print("\nSLEEP TEST 1")
unix.sleep(1)
print("\nSLEEP TEST 2")
unix2.sleep(1)
--]]

print("\nUNAME TEST 1")
print(unix.uname())
print("\nUNAME TEST 2")
print(unix2.uname())

print("\n HOSTNAME TEST 1")
print(unix.gethostname())
print("\n HOSTNAME TEST 2")
print(unix2.gethostname())

print("\n GETCWD TEST 1")
local path0 = unix.getcwd()
print(path0)
print("\n GETCWD TEST 2")
local path1 = unix2.getcwd()
print(path1)

print("\n CHDIR TEST 1")
print(unix.chdir('/'))
print(unix.chdir(path0))
print("\n CHDIR TEST 2")
print(unix2.chdir('/'))
print(unix2.chdir(path1))

print("\n OPEN TEST 1")
local fd0 = unix.open("Makefile", unix.O_RDONLY)
print(fd0)
print("\n OPEN TEST 2")
local fd1 = unix2.open("Makefile", unix2.O_RDONLY)
print(fd1)

print(' \nREAD CAT')
os.execute("cat Makefile")
print("\n READ TEST 1")
local ret0 = unix.read(fd0)
print(ret0)
print("\n READ TEST 2")
local ret1 = unix2.read(fd1)
print(ret1)

print("\n CLOSE TEST 1")
print(unix.close(fd0))
print("\n CLOSE TEST 2")
print(unix2.close(fd1))

print("\n READDIR TEST 1")
local dir0 = unix.readdir(path0)
util.ptable(dir0)
print("\n READDIR TEST 2")
local dir1 = unix2.readdir(path1)
util.ptable(dir1)
for k, v in pairs(dir0) do
	local found = false
	for kk, vv in pairs(dir1) do
		if v==vv then found = true end
	end
	assert(found)
end

print("\n MKFIFO TEST 1")
print(unix.mkfifo('/tmp/test1', 438))
print("\n MKFIFO TEST 2")
print(unix2.mkfifo('/tmp/test2', 438))

print("\n SELECT TEST 1")
local status0, available0 = unix.select({}, 1)
print(status0)
util.ptable(available0)
print("\n SELECT TEST 2")
local status1, available1 = unix2.select({}, 1)
print(status1)
util.ptable(available0)