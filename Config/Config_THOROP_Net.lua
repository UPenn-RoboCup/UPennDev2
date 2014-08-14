assert(Config, 'Need a pre-existing Config table!')

-- Network settings --
local net = {
	use_wireless = false,
}

-- IP Addresses
-- TODO: Find IP of this computer
if Config.use_localhost then	
  net.robot = {
		['wired']    = 'localhost',
		['wireless'] = 'localhost',
	}
	net.operator = {
		['wired']              = 'localhost',
		['wired_broadcast']    = 'localhost',
		--
		['wireless']           = 'localhost',
		['wireless_broadcast'] = 'localhost',
	}
else
	net.robot = {
		['wired']    = '192.168.123.24',
		['wireless'] = '192.168.1.24',
	}
	net.operator = {
		['wired']              = '192.168.123.23',
		['wired_broadcast']    = '192.168.123.255',
		--
		['wireless']           = '192.168.1.23',
		['wireless_broadcast'] = '192.168.1.255'
	}
end

-- Ports for Remote Procedure Calls
net.rpc = {
	tcp_reply = 55555,
	tcp_subscribe = 55558,
	udp = 55556,
	uds = 'rpc',
}

local streams = {}
net.streams = streams
streams.mesh = {
	ws = 9001,
	udp = 33344,
	tcp = 43344,
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
streams.kinect_depth = {
  ws = 9010,
	udp= 33346,
}
streams.kinect_color = {
  ws = 9011,
	udp= 33347,
}
streams.feedback = {
  ws = 9013,
	udp= 54329,
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
