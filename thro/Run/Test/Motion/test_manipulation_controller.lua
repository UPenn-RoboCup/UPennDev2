dofile('../../include.lua')

require('mcm')
require('pcm')
require('dcm')
require('stand')
require('curses')
require('Config')
require('Platform')
require('Transform')
require('Proprioception')
require('manipulation_controller')
require('Locomotion')

Platform.set_update_rate(500)
Platform.entry()
Locomotion:entry()
Locomotion:add_event('stand')
Locomotion:set_joint_access(0, 'upperbody')

function draw_screen()
  curses.clear()
  curses.printw('rate: %7.2f            Test Manipulation Controller\n', 
    Platform.get_update_rate())
  curses.printw('///////////////////////////////////////')
  curses.printw('///////////////////////////////////////\n')
  curses.printw('controls\n')
  curses.printw('---------------------------------------')
  curses.printw('---------------------------------------\n')
  curses.printw('---------------------------------------')
  curses.printw('---------------------------------------\n')
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

while (true) do
  Platform.update()
  Locomotion:update()

  -- handle keystrokes
  local key = curses.getch()
  if (key) then
    if (key == string.byte('q')) then
      break
    elseif (key == string.byte(' ')) then
    elseif (key == string.byte('i')) then
    elseif (key == string.byte(',')) then
    elseif (key == string.byte('k')) then
    elseif (key == string.byte('j')) then
    elseif (key == string.byte('l')) then
    elseif (key == string.byte('h')) then
    elseif (key == string.byte(';')) then
    elseif (key == curses.KEY_RESIZE) then
      local y, x = curses.getmaxyx()
      curses.resizeterm(y, x)
    end
  end

  -- update screen
  loop_count = loop_count + 1
  if (loop_count % 20 == 0) then
    draw_screen()
  end
end

Locomotion:exit()
curses.endwin()
