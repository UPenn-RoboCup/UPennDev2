dofile('include.lua')

--------------------------------------------------------------------------------
-- Walk Command
--------------------------------------------------------------------------------

require('rpc')
require('curses')
require('vector')

-- connect to motion manager
local motion_manager_endpoint = 'tcp://localhost:12001'
local motion_manager = rpc.client.new(motion_manager_endpoint)
print('Attempting to connect to motion_manager at '..motion_manager_endpoint)
motion_manager:connect(nil)
motion_manager:set_timeout(0.1)

function draw_screen()
  curses.clear()
  curses.printw('                                Walk Command\n')
  curses.printw('///////////////////////////////////////')
  curses.printw('///////////////////////////////////////\n')
  curses.printw('controls\n')
  curses.printw('---------------------------------------')
  curses.printw('---------------------------------------\n')
  curses.printw('i         : forward\n')
  curses.printw(',         : back\n')
  curses.printw('j         : turn right\n')
  curses.printw('l         : turn left\n')
  curses.printw('h         : side right\n')
  curses.printw(';         : side left\n')
  curses.printw('k         : zero velocity\n')
  curses.printw('space_bar : start / stop\n')
  curses.printw('---------------------------------------')
  curses.printw('---------------------------------------\n')
  curses.refresh()
end

function update_display()
  local success, state = motion_manager:call('Locomotion:get_state')
  if (success) then
    curses.move(13, 0)
    curses.printw('state     : %s              ', state)
  end
  local success, velocity = motion_manager:call('walk:get_velocity')
  if (success) then
    curses.move(14, 0)
    curses.printw('velocity  :%7.4f %7.4f %7.4f', unpack(velocity))
  end
  curses.refresh()
end

-- initialize display
curses.initscr()
curses.cbreak()
curses.noecho()
curses.keypad(1)
curses.timeout(1)
draw_screen()

-- initialize control variables
local loop_count = 0
local desired_velocity = vector.new{0, 0, 0}

while (true) do

  -- handle keystrokes
  local key = curses.getch()
  if (key) then
    if (key == string.byte('q')) then
      break
    elseif (key == string.byte(' ')) then
      local success, state = motion_manager:call('Locomotion:get_state')
      desired_velocity = vector.new{0, 0, 0}
      if (state ~= 'stand') then 
	motion_manager:call('Locomotion:add_event', 'stand')
      else
	motion_manager:call('Locomotion:add_event', 'walk')
      end
    elseif (key == string.byte('i')) then
      desired_velocity = desired_velocity + vector.new{0.005, 0, 0} 
    elseif (key == string.byte(',')) then
      desired_velocity = desired_velocity + vector.new{-0.005, 0, 0} 
    elseif (key == string.byte('k')) then
      desired_velocity = vector.new{0, 0, 0}
    elseif (key == string.byte('j')) then
      desired_velocity = desired_velocity + vector.new{0, 0, 0.005}
    elseif (key == string.byte('l')) then
      desired_velocity = desired_velocity + vector.new{0, 0,-0.005}
    elseif (key == string.byte('h')) then
      desired_velocity = desired_velocity + vector.new{0, 0.005, 0}
    elseif (key == string.byte(';')) then
      desired_velocity = desired_velocity + vector.new{0,-0.005, 0}
    end
    motion_manager:call('walk:set_velocity', unpack(desired_velocity))
    update_display()
  end

  -- update screen
  loop_count = loop_count + 1
  if (loop_count % 20 == 0) then
    update_display()
  end
end

motion_manager:close()
curses.endwin()
