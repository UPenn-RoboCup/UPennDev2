dofile('include.lua')

----------------------------------------------------------------------
-- Demo
----------------------------------------------------------------------

require('unix')
require('util')
require('Body')
require('walk')
require('curses')
require('Config')
require('Motion')
require('Locomotion')

require 'rcm'
require 'WebotsLaser'

Body.entry()
Motion.add_fsm(Locomotion)
Locomotion:add_event('walk')

function draw_screen()
  curses.clear()
  curses.printw('fps : %.2f     \n', Body.get_update_rate())
  curses.printw('===================================\n')
  curses.printw('             SLAMMER\n')
  curses.printw('===================================\n')
  curses.printw('controls\n')
  curses.printw('-----------------------------------\n')
  curses.printw('i : forward\n')
  curses.printw(', : back\n')
  curses.printw('j : turn right\n')
  curses.printw('l : turn left\n')
  curses.printw('h : side right\n')
  curses.printw('; : side left\n')
  curses.printw('k : zero velocity\n')
  curses.printw('space : start and stop\n')
  curses.printw('-----------------------------------\n')
  curses.refresh()
end

function update_display()
  curses.move(0, 0)
  curses.printw('fps : %.2f     \n', Body.get_update_rate())
  curses.move(15, 0)
  curses.printw('velocity : %7.4f %7.4f %7.4f', unpack(walk:get_velocity()))
  curses.move(16, 0)
  curses.printw('timescan : %7.4f', rcm:get_timestamp(1) )
  curses.refresh()
end

curses.initscr()
curses.cbreak()
curses.noecho()
curses.keypad(1)
curses.timeout(1)
draw_screen()

local count = 0

while true do

  Body.update()
  Motion.update()
	
	-- Update the laser scanner
	local lidar_scan = WebotsLaser.get_scan()
	-- Set the Range Comm Manager values
	rcm:set_ranges( carray.pointer(lidar_scan) );
	rcm:set_timestamp( Body.get_time() )

  -- handle keystrokes
  local key = curses.getch()
  if (key == string.byte('i')) then
    walk:set_velocity(unpack(walk:get_velocity() + vector.new{0.005, 0, 0}))
  elseif (key == string.byte(',')) then
    walk:set_velocity(unpack(walk:get_velocity() + vector.new{-0.005, 0, 0}))
  elseif (key == string.byte('k')) then
    walk:set_velocity(0, 0, 0)
  elseif (key == string.byte('j')) then
    walk:set_velocity(unpack(walk:get_velocity() + vector.new{0, 0, 0.005}))
  elseif (key == string.byte('l')) then
    walk:set_velocity(unpack(walk:get_velocity() + vector.new{0, 0, -0.005}))
  elseif (key == string.byte('h')) then
    walk:set_velocity(unpack(walk:get_velocity() + vector.new{0, 0.005, 0}))
  elseif (key == string.byte(';')) then
    walk:set_velocity(unpack(walk:get_velocity() + vector.new{0, -0.005, 0}))
  elseif (key == string.byte(' ')) then
    if walk:is_active() then
      walk:stop() 
    else
      walk:start()
    end
  elseif (key == string.byte('q')) then
    break
  end
  -- update screen
  count = count + 1
  if (key or (count % 20 == 0)) then
    update_display()
  end
end

curses.endwin()
Motion.exit()
Body.exit()
