dofile'include.lua'
local Body = require'Body'
local getch = require'getch'
local util = require'util'
local vector = require'vector'
local color = util.color
local running = true
local tasks = {}
local t0 = unix.time()




local t_last_temp = 0
local last_position = nil
local t0 = Body.get_time()
local t_blink = 2.0

function display_led(rgb,intensity)
  	Body.set_head_led_red(rgb[1]*intensity)
  	Body.set_head_led_blue(rgb[2]*intensity)
  	Body.set_head_led_green(rgb[3]*intensity)
end



while running do
  local t = Body.get_time()
  local ph = (t-t0)/t_blink
  local  current_lidar_deg = Body.get_lidar_position()[1] * RAD_TO_DEG
  local cur_position, rgb
  local intensity = 1.0

  print(current_lidar_deg)
  
  if current_lidar_deg>45 then
    cur_position=1
    rgb={255,0,0}
  elseif current_lidar_deg<-45 then
    cur_position = -1
    rgb={0,0,255}
  else
    cur_position = 0
    rgb={255,0,255}
  end

  if cur_position~=last_position then
    last_position = cur_position
    display_led(rgb,intensity)
  end

  unix.usleep(1e5)
end
