assert(Config, 'Need a pre-existing Config table!')

-- IP addresses
local IP = {
  STEVE = 23,
  SJ = 200,
  BHORAM = 54,
  JQ = 150,
--
  ALVIN = 232,
  FIELD = 242,
}

-- Who do we use?
local WHO = IP.STEVE
local ROBOT_IP = IP.ALVIN
local TEAM_NUMBER = 8

local wired_subnet = '192.168.123.'
local wireless_subnet = '192.168.1.'

local net = {
	field_computer = wired_subnet..IP.FIELD,
	robot = {
		wired = wired_subnet..ROBOT_IP,
		wireless = wireless_subnet..ROBOT_IP,
	},
	operator = {
		wired              = wired_subnet..WHO,
		wireless           = wireless_subnet..WHO,
	},
	broadcast = {
		wired = wired_subnet..'255',
		wireless = wireless_subnet..'255'
	}
}

if Config.IS_COMPETING then
	net.field_computer = '10.'..TEAM_NUMBER..'.3.'..IP.FIELD
	--
	net.robot.wired = '10.'..TEAM_NUMBER..'.3.'..ROBOT_IP
	net.robot.wireless = net.robot.wired
	--
	net.operator.wired = '10.'..TEAM_NUMBER..'.2.'..ROBOT_IP
	net.operator.wireless = net.operator.wired
	-- Broadcast from the robot to the operator(s)
	net.broadcast.wired = '10.'..TEAM_NUMBER..'.2.'..ROBOT_IP
	net.broadcast.wireless = net.broadcast.wired
end


-- Check lossy link
net.test = {
	udp = 16999, -- unreliable
	tcp = 1999 -- reliable
}

local streams = {}
net.streams = streams
-- Lossless link
net.rpc = {
	tcp_reply = 2000,
	udp = 2001,
	uds = 'rpc',
}
streams.feedback = {
  ws = 9013,
	udp = 2002,
	sub = 'feedback'
}
-- Lossy Link
streams.camera0 = {
  ws = 9003,
  udp = 17000,
	sub = 'camera0',
}
streams.mesh = {
	ws = 9001,
	udp = 17001,
	tcp = 43344,
  sub = 'mesh0'
}
streams.kinect2_depth = {
  ws = 9010,
	udp = 17002,
  tcp = 43346,
	sub = 'kinect2_depth'
}
streams.kinect2_color = {
  ws = 9011,
	udp = 17003,
  tcp = 43347,
	sub = 'kinect2_color'
}
--[[
streams.camera1 = {
  ws = 9004,
  udp= 33334,
	sub = 'camera1',
}
--]]
streams.audio = {
  ws = 9014,
	tcp= 55557,
}
--[[
streams.lidar0 = {
  ws = 9015,
	sub = 'lidar0',
}
--]]

-- Export
Config.net = net

return Config
