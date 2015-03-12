assert(Config, 'Need a pre-existing Config table!')

-- IP addresses
local IP = {
  STEVE = 23,
  SJ = 200,
  KAREN = 30,
  BHORAM = 54,
  ALVIN = 24,
  ALVIN_Z = 222,
  ALVIN_B = 232,
  TEDDY = 24,
  FIELD = 201,
}

-- Who do we use?
local WHO = IP.STEVE
local ROBOT_IP = IP.TEDDY
local TEAM_NUMBER = 8

local subnets = {
	OPERATOR = '192.168.123.',
	FIELD = '192.168.123.',
	ROBOT = '192.168.123.',
}
if Config.IS_COMPETING then
	subnets = {
		OPERATOR = '10.'..TEAM_NUMBER..'.2.',
		FIELD = '10.'..TEAM_NUMBER..'.3.',
		ROBOT = '10.'..TEAM_NUMBER..'.3.'
	}
end

local net = {
	use_wireless = false,
	field_computer = subnets.FIELD..IP.FIELD,
}

-- IP Addresses
-- TODO: Find IP of this computer
if Config.use_localhost then
  net.robot = {
		wired   = 'localhost',
		wireless = 'localhost',
	}
	net.operator = {
		wired              = 'localhost',
		['wired_broadcast']    = 'localhost',
		--
		wireless           = 'localhost',
		['wireless_broadcast'] = 'localhost',
	}
else
	net.robot = {
		wired    = subnets.ROBOT..ROBOT_IP,
		wireless = '192.168.1.'..ROBOT_IP,
	}
	net.operator = {
		wired              = subnets.OPERATOR..WHO,
		['wired_broadcast']    = subnets.OPERATOR..'255',
		--
		wireless           = '192.168.1.'..WHO,
		['wireless_broadcast'] = '192.168.1.255'
	}
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
