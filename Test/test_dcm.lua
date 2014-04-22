dofile'../include.lua'
local simple_ipc = require'simple_ipc'
local chain = {
	device = '/dev/cu.usbserial-FTVTLUY0B',
	m_ids = {2,4,6,8},
}
local ch, thread = 
	simple_ipc.new_thread(ROBOT_HOME..'dcm.lua','dcm',chain)
ch.callback = function() print'hi' end
thread:start()
poller = simple_ipc.wait_on_channels({ch})
poller:start()
