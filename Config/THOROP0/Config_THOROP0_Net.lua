assert(Config, 'Need a pre-existing Config table!')

-- IP addresses
local IP = {
	STEVE = 23,
	SJ = 200,
	BHORAM = 57,
	JQ = 150,
	--
	CHIP = 245,
	DALE = 246,
	FIELD = 242,
}

-- Who do we use?
local WHO = IP.STEVE
local ROBOT_IP = IP.DALE
local TEAM_NUMBER = 8

--local wired_subnet = '192.168.123.'
local wired_subnet = '10.8.2.'
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

if IS_COMPETING then
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

	net.ping = {
		udp = 17000,
		tcp = 2000,
		pub = 'ping',
		sub = 'go',
	}

end

local streams = {}
net.streams = streams
-- Lossless link
net.rpc = {
	tcp_reply = 2000,
	udp = 2001,
	uds = 'rpc',
}
streams.feedback = {
	ws = 9002,
	udp = 2002,
	sub = 'feedback'
}
streams.ittybitty0 = {
	ws = 9070,
	udp = 2030,
	sub = 'ittybitty0'
}
streams.ittybitty1 = {
	ws = 9071,
	udp = 2031,
	sub = 'ittybitty1'
}
-- Lossy Link
streams.camera0 = {
	ws = 9003,
	udp = 17003,
	tcp = 43303,
	sub = 'camera0',
}
streams.camera1 = {
	ws = 9004,
	udp = 17004,
	tcp = 43304,
	sub = 'camera1',
}
streams.lidar0 = {
	ws = 9010,
	udp = 17010,
	tcp = 43310,
	sub = 'lidar0'
}
streams.lidar1 = {
	ws = 9011,
	udp = 17011,
	tcp = 43311,
	sub = 'lidar1'
}
streams.mesh0 = {
	ws = 9020,
	udp = 17020,
	tcp = 43300,
	sub = 'mesh0'
}
streams.mesh1 = {
	ws = 9021,
	udp = 17021,
	tcp = 43301,
	sub = 'mesh1'
}
streams.kinect2_depth = {
	ws = 9046,
	udp = 17046,
	tcp = 43346,
	sub = 'kinect2_depth'
}
streams.kinect2_color = {
	ws = 9047,
	udp = 17047,
	tcp = 43347,
	sub = 'kinect2_color'
}
--[[
streams.audio = {
ws = 9014,
tcp= 55557,
}
--]]

-- Export
Config.net = net

return Config
