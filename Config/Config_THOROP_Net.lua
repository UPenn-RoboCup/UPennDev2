assert(Config, 'Need a pre-existing Config table!')
local vector = require'vector'

----------------------
-- Network settings --
----------------------
local net = {}
-- Use wired or wireless
--net.use_wireless = false
net.use_wireless = true

-- Robot IP addresses
net.robot = {
['wired']    = '192.168.123.24',
['wireless'] = '192.168.136.24',
}

-- Remote Operator IP addresses
net.operator = {
--['wired']              = '192.168.123.23', --Steve's
--['wired']              = '192.168.123.200', --SJ's
['wired']              = '192.168.123.30',  --Karen's
['wired_broadcast']    = '192.168.123.255',
--
['wireless']           = '192.168.136.30',
['wireless_broadcast'] = '192.168.136.255'
}

-- For use only when testing in webots on a local computer
if Config.use_localhost then	
  -- wired
  net.robot.wired = 'localhost'
  net.operator.wired = 'localhost'
  net.operator.wired_broadcast = 'localhost'
  -- wireless
  net.robot.wireless = 'localhost'
  net.operator.wireless = 'localhost'
  net.operator.wireless_broadcast = 'localhost'
end

-- Ports
net.reliable_rpc   = 55555 -- REP
net.unreliable_rpc = 55556 -- UDP
net.audio = 55557
net.reliable_rpc2  = 55558 -- SUB
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

-- Export
Config.net = net

return Config
