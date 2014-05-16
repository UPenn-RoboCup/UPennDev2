dofile'../include.lua'
local TEST_THREAD = true
--local TEST_BODY = true

if TEST_THREAD then
	local simple_ipc = require'simple_ipc'
	local ch, thread =
		simple_ipc.new_thread(ROBOT_HOME..'/dcm.lua','dcm')
	ch.callback = function(ch)
		local msg = ch:receive()
		if msg=='exit' then poller:remove(ch) end
	end
	thread:start()
	poller = simple_ipc.wait_on_channels{ch}
	poller:start()
  print('DONE')
end

if TEST_BODY then
	local Body = require(Config.dev.body)
	local t_sleep = Body.update_cycle * 1e6
	Body.entry()
	while true do
		Body.update()
		unix.usleep(t_sleep)
	end
	Body.exit()
print('DONE')
end
