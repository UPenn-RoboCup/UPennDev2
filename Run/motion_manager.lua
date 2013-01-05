dofile('include.lua')

----------------------------------------------------------------------
-- Motion Manager
----------------------------------------------------------------------

require('rpc')
require('unix')
require('util')
require('Body')
require('curses')
require('Config')
require('Motion')
require('Locomotion')
require('Manipulation')
require('Proprioception')

local function draw_screen()
  curses.clear()
  curses.printw('                              Motion Manager\n')
  curses.printw('///////////////////////////////////////')
  curses.printw('///////////////////////////////////////\n')
  curses.printw('update rate : %.2f     \n', Body.get_update_rate())
  curses.refresh()
end

-- initialize motion state machines
Body.entry()
Proprioception.entry()
Motion.entry()
Motion.add_fsm(Locomotion)
Motion.add_fsm(Manipulation)
Locomotion:add_event('walk')

-- initialize screen
curses.initscr()
curses.cbreak()
curses.noecho()
curses.keypad(1)
curses.timeout(1)
draw_screen()

local count = 0
local motion_rpc_server = rpc.new_server('MOTION')

while true do
  -- handle remote procedure calls
  motion_rpc_server:update()

  -- update controllers
  Body.update()
  Proprioception.update()
  Motion.update()

  -- handle keystrokes
  local key = curses.getch()
  if(key == string.byte('q')) then
    break
  end

  -- update screen
  count = count + 1
  if (key or (count % 20 == 0)) then
    draw_screen()
  end
end

curses.endwin()
Motion.exit()
Proprioception.exit()
Body.exit()
