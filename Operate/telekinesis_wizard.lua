-----------------------------------------------------------------
-- Telekinesis Wizard
-- Listens to spacemouse input to control 
-- the position/orientation of virtual object
-- (c) Stephen McGill, 2013
---------------------------------

dofile'include.lua'

-- Memory access
require'tkcm'

-- Libraries
local unix = require'unix'
local vector = require'vector'
local quaternion = require'quaternion'
local spacemouse = require 'spacemouse'
local getch = require'getch'
local colors = require'colors'
--sm = spacemouse.init(0x046d, 0xc62b) -- pro
local sm = spacemouse.init(0x046d, 0xc626) -- regular
-- Update every 10ms
local update_interval = 0.010 * 1e6

local current_obj = 1
local objects = {'table','drill'}

local function process_button(btn)
  if btn==1 then
    current_obj = current_obj - 1
    if current_obj<1 then current_obj=#objects end
    return'left'
  elseif btn==2 then
    current_obj = current_obj + 1
    if current_obj>#objects then current_obj=1 end
    return'right'
  elseif btn==0 then
    return'none'
  elseif btn==3 then
    return'both'
  end
  return'bad button!'
end

local trans_scale = 0.01 / 350
local function process_translate( data )
  local delta_pos = trans_scale * vector.new({data.x,data.y,data.z,0,0,0})
  -- Don't move if the input is small
  if vector.norm(delta_pos)<0.003 then return nil end
  local cur_pos = tkcm['get_'..objects[current_obj]..'_position']()
  tkcm['set_'..objects[current_obj]..'_position'](cur_pos+delta_pos)
end

local rot_scale = 1 * (math.pi/180)/350
local function process_rotate(data)
  local q_w = rot_scale * vector.new({data.wx,data.wy,data.wz})
  local d_q = quaternion.new(q_w)
  local q_cur = tkcm['get_'..objects[current_obj]..'_orientation']()
  local q_new = quaternion.unit( d_q*q_cur )
  tkcm['set_'..objects[current_obj]..'_orientation'](q_new)
end

local function direct_rotate(dat)
  local current = get_joint()
  local new = current - rot_scale * data.wz
  set_joint(new)
  return''
end

------------
-- Start processing
os.execute("clear")
io.flush()
local msg = 'Unknown'
local t0 = unix.time()
while true do
  local t = unix.time()
  local evt, data = sm:get()
  if evt=='button' then process_button(data) end
  if evt=='rotate' then process_rotate(data) end
  if evt=='translate' then process_translate(data) end
  -- Print result of the key press
  local t_diff = t-t0
  if t_diff>1 then
    os.execute("clear")
    local obj_name = objects[current_obj] or 'nothing'
    print(colors.wrap('Moving the '..obj_name..'...','yellow'))
    t0 = t
  end
  
  -- Grab the keyboard character
  local key_code = getch.nonblock()
  if key_code then
    local key_char = string.char(key_code)
    local key_char_lower = string.lower(key_char)
    print('Key',key_code,key_char,key_char_lower)
  end
  
  unix.usleep( update_interval )
end
