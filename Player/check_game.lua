#!/usr/bin/env lua

dofile'../include.lua'
local util = require'util'

local processes = {
  'run_co_dcm',
  'run_imu',
  'state_wizard',
}
local is_running = {}
for _, v in ipairs(processes) do
  is_running[v] = false
end

x = io.popen("pgrep -fl luajit")

for l in x:lines() do
  local m = l:match("luajit%s+(.+)")
  for _, p in ipairs(processes) do
    if m:find(p) then
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
