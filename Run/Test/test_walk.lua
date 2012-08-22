dofile('../include.lua')

----------------------------------------------------------------------
-- Test walk 
----------------------------------------------------------------------

require('unix')
require('util')
require('Body')
require('walk')
require('getch')
require('curses')
require('Config')

Body.entry()

walk:entry()
walk:start()

local stats = util.loopstats(20)

function draw_screen()
  curses.clear()
  curses.printw('fps : %.2f\n', stats.fps())
  curses.printw('===================================\n')
  curses.printw('              Test Walk\n')
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
  curses.printw('velocity : %7.4f %7.4f %7.4f', unpack(walk:get_velocity()))
  curses.refresh()
end

curses.initscr()
curses.cbreak()
curses.noecho()
curses.keypad(1)
curses.timeout(0)
draw_screen()

while true do

  Body.update()
  walk:update()

  -- handle keystrokes
  local key = curses.getch()
  if (key == string.byte('i')) then
    walk:set_velocity(unpack(walk:get_velocity() + vector.new{0.001, 0, 0}))
  elseif (key == string.byte(',')) then
    walk:set_velocity(unpack(walk:get_velocity() + vector.new{-0.001, 0, 0}))
  elseif (key == string.byte('k')) then
    walk:set_velocity(0, 0, 0)
  elseif (key == string.byte('j')) then
    walk:set_velocity(unpack(walk:get_velocity() + vector.new{0, 0, 0.001}))
  elseif (key == string.byte('l')) then
    walk:set_velocity(unpack(walk:get_velocity() + vector.new{0, 0, -0.001}))
  elseif (key == string.byte('h')) then
    walk:set_velocity(unpack(walk:get_velocity() + vector.new{0, 0.001, 0}))
  elseif (key == string.byte(';')) then
    walk:set_velocity(unpack(walk:get_velocity() + vector.new{0, -0.001, 0}))
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
  if (key or stats.update()) then
    draw_screen()
  end
end

Body.exit()
