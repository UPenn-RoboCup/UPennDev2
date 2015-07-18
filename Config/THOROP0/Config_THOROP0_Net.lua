assert(Config, 'Need a pre-existing Config table!')

-- IP addresses
local IP = {
	STEVE = 23,
	SJ = 201,
	ASUS = 200,
	BHORAM = 57,
	JQ = 150,
	--
	CHIP2 = 144,
	CHIP = 145,
	DALE = 146,
	FIELD = 132,
}

-- Who do we use?
local WHO = IP.STEVE
local WHO = IP.SJ
local ROBOT_IP = IP.CHIP
local TEAM_NUMBER = 8

local net = {
	robot = {
		wireless = '192.168.123.'..ROBOT_IP,
		wired = '192.168.123.'..ROBOT_IP,
	},
	operator = {
		wireless = '192.168.123.'..WHO,
		wired = '192.168.123.'..WHO,
	},
	broadcast = {
		wireless = '192.168.123.255',
		wired = '192.168.123.255',
	}
}

-- DRC Finals Specific
--[[
net.field_computer = '10.'..TEAM_NUMBER..'.3.'..IP.FIELD
-- Robot IP
net.robot.wired = '10.'..TEAM_NUMBER..'.3.'..ROBOT_IP
-- Operator IP
net.operator.wired = '10.'..TEAM_NUMBER..'.2.'..WHO
-- Broadcast from the robot to the operator(s)
net.broadcast.wired = '10.'..TEAM_NUMBER..'.2.'..WHO
--]]

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
	--tcp = 43200,
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
streams.vision0 = {
	ws = 9013,
	udp = 17013,
	tcp = 43313,
	sub = 'vision0',
}
streams.label = {
	ws = 9014,
	udp = 17014,
	tcp = 43314,
	sub = 'label',
}
streams.world = {
	ws = 9023,
	udp = 17023,
	tcp = 43323,
	sub = 'world',
}
streams.lidar0 = {
	ws = 9010,
	udp = 17010,
	--tcp = 43310,
	sub = 'lidar0'
}
streams.lidar1 = {
	ws = 9011,
	udp = 17011,
	--tcp = 43311,
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
--[[
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
--]]
--[[
streams.audio = {
ws = 9017,
tcp= 55557,
}
--]]

-- Export
Config.net = net

return Config
