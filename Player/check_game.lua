#!/usr/bin/env lua

dofile'../include.lua'
local util = require'util'

local processes = {
  'run_dcm',
  'run_imu',
  'camera_wizard',
  'world_wizard',
  'state_wizard',
  'gc_wizard',
}
local is_running = {}
for _, v in ipairs(processes) do
  is_running[v] = false
end

local pgrep = io.popen("pgrep -fl luajit")

for l in pgrep:lines() do
  local m = l:match("luajit%s+(.+)")
  for _, p in ipairs(processes) do
    if m and m:find(p) then
      is_running[p] = true
    end
  end
end

local is_done = true
for k, v in pairs(is_running) do
  if not v then
    is_done = false
    print("NOT RUNNING", util.color(k, 'red'))
  else
    print(util.color(k, 'green'))
  end
end

if not is_done then
  print("You are not ready for a game!")
  os.exit(1)
end
