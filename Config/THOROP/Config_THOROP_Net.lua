assert(Config, 'Need a pre-existing Config table!')

-- IP addresses
local IP = {
  STEVE = 23,
  SJ = 200,
  KAREN = 30,
  BHORAM = 54,
  ALVIN = 24,
  ALVIN_Z = 222,
  TEDDY = 26,
}

-- Who do we use?
--local WHO = IP.BHORAM
local WHO = IP.STEVE
local WHICH = IP.ALVIN_Z
--local WHICH = IP.TEDDY

-- Network settings --
local net = {
	use_wireless = false,
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
		wired    = '192.168.123.'..WHICH,
		wireless = '192.168.1.'..WHICH,
	}
	net.operator = {
		wired              = '192.168.123.'..WHO,
		['wired_broadcast']    = '192.168.123.255',
		--
		wireless           = '192.168.1.'..WHO,
		['wireless_broadcast'] = '192.168.1.255'
	}
end

-- Ports for Remote Procedure Calls
net.rpc = {
	tcp_reply = 55555,
	udp = 55556,
	uds = 'rpc',
}

local streams = {}
net.streams = streams
streams.mesh = {
	ws = 9001,
	udp = 33344,
	tcp = 43344,
  sub = 'mesh0'
}
streams.camera0 = {
  ws = 9003,
  udp= 33333,
	sub = 'camera0',
}
streams.camera1 = {
  ws = 9004,
  udp= 33334,
	sub = 'camera1',
}
streams.lidar0 = {
  ws = 9015,
	sub = 'lidar0',
}
streams.kinect2_depth = {
  ws = 9010,
	udp = 33346,
  tcp = 43346,
	sub = 'kinect2_depth'
}
streams.kinect2_color = {
  ws = 9011,
	udp = 33347,
  tcp = 43347,
	sub = 'kinect2_color'
}
streams.feedback = {
  ws = 9013,
	udp= 54329,
	sub = 'feedback'
}
streams.audio = {
  ws = 9014,
	tcp= 55557,
}

-- TODO: Eliminate these
net.team           = 44444
net.state          = 44445

-- Export
Config.net = net

return Config
