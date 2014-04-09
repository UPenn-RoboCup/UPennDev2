dofile'../../include.lua'
local DO_LOG = true
local libLog, logger
if DO_LOG then
	libLog = require'libLog'
	-- For timing
	require'unix'
	-- Make the logger
	logger = libLog.new('uvc',true)
end

local uvc = require'uvc'
video_ud1 = uvc.init('/dev/video0', 640, 480, 'yuyv',1, 30)

local n, t = 0, unix.time()
while n<5 do
	n = n+1
	t = unix.time()
	local img1, size1, count1, time1 = video_ud1:get_image();
	if (img1 ~= -1) then
		print('img1', img1, size1, time1, count1)
		print(type(img1))
		if DO_LOG then
			local m_ok, r_ok = logger:record({t=t,n=i},img1,size1)
			print(n, m_ok, r_ok)
			-- Wait half a second
			unix.usleep(5e5)
		end
	end
end
logger:stop()
