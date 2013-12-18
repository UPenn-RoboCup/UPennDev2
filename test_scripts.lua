dofile'include.lua'

signal = require'signal'
require'unix'

function shutdown(info)
  print('Info',info)
  os.exit()
end

signal.signal("SIGINT", shutdown)
signal.signal("SIGTERM", shutdown)

print('start!')
while true do
unix.usleep(1e4)
end
