dofile'include.lua'
local Body = require'Body'
local mp = require'msgpack'
local fps = 10
local twait_usec = 1/fps * 1e6
local f = io.open('../Logs/rarm2.log','w')
local n_sec = 10 -- record for 10 seconds
local n = 0
local t0 = unix.time()
local t_last = t0
local cache = ''
while true do
  n = n + 1
  local t = unix.time()
  local larm_cmd = Body.get_rarm_command_position()
  table.insert(larm_cmd,t)
  cache = cache..mp.pack(larm_cmd)
  local t_diff = t-t_last

  -- Debug and write the cache to disk
  if t_diff>1 then
    f:write(cache)
    --if t-t0>n_sec then print('Done!'); return; end
    print('Writing',n)
    cache = ''
    t_last = unix.time()
    twait_usec = twait_usec - 1e6*(t_last-t)
  end

  unix.usleep(twait_usec)
end
