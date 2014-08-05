assert(Config, 'Need a pre-existing Config table!')
local vector = require'vector'

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
--[[
net.audio = 55557
--
net.team           = 44444
net.state          = 44445
--
net.camera = {}
net.camera.head     = 33333
net.camera.forehead = 33335
net.camera.forehead2 = 33337
net.detect = 33331
--
net.reliable_camera = {}
net.reliable_camera.head     = 33334
net.reliable_camera.forehead = 33336
net.reliable_camera.forehead2 = 33338
--
net.mesh           = 33344
net.reliable_mesh  = 33345
net.rgbd_depth     = 33346
net.rgbd_color     = 33347
--
net.omap           = 22222
net.hmap           = 22223

net.feedback       = 54329


mesh = {
	name : 'mesh', // reliable name
	ws : 9001,
	udp: 33344,
}

bridges.push({
  name : 'camera0',
  ws : 9003,
  udp: 33333,
  //tcp : 33334,
	sub : 'camera0',
  clients : []
});

bridges.push({
  name : 'camera1',
  ws : 9004,
  udp: 33335,
  //tcp: 33336,
  clients : []
});

bridges.push({
	name : 'feedback',
	ws : 9013,
	udp: 54329,
	clients : []
});

bridges.push({
	name : 'audio',
	ws : 9014,
  //tcp: 55557,
	clients : []
});

bridges.push({
	name : 'lidar0',
	ws : 9015,
	sub: 'lidar0',
	clients : []
});

bridges.push({
	name : 'touch',
	ws : 9064,
	pub: 'touch',
	sub: 'bbox', // listen for the bounding box
	/*tcp: '55588',*/
	clients : []
});

bridges.push({
	name : 'wire',
	ws : 9065,
	sub: 'wire',
	tcp: '55589',
	clients : []
});

/*
bridges.push({
	name : 'rgbd_depth',
	ws : 9010,
	udp: 33346,
	clients : []
});

bridges.push({
  name : 'rgbd_color',
  ws : 9011,
  udp: 33347,
  clients : []
});

bridges.push({
	name : 'spacemouse',
	ws : 9012,
	sub: 'spacemouse',
	clients : []
});
*/
--]]


-- Export
Config.net = net

return Config
