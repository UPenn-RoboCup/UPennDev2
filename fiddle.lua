dofile'include.lua'
-- Put unix into the global space
unix = require'unix'
local listing = unix.readdir(HOME..'/Player')
-- Add all FSM directories that are in Player
local simple_ipc = require'simple_ipc'
for _,sm in ipairs(listing) do
  local found = sm:find'FSM'
  if found then
    -- make GameFSM to game_ch
    local name = sm:sub(1,found-1):lower()
    print(sm,name)
    -- Put into the global space
    _G[name] = simple_ipc.new_publisher(sm,true)
  end
end
-- Add all shm
local listing = unix.readdir(HOME..'/Memory')
for _,mem in ipairs(listing) do
  local found, found_end = mem:find'cm'
  if found then require(mem:sub(1,found_end)) end
end