dofile('../include.lua')

require('dcm')
require('unix')
require('curses')

-- log load cell forces from linear actuators
--------------------------------------------------------------------------

local key = ''
local t0 = unix.time()
local t  = unix.time()
local delay = 3e3 -- microseconds
local filename = arg[1] or 'forces.txt' 
local f = assert(io.open(_HOME_..'/logs/'..filename, 'w+'))

curses.initscr()
curses.cbreak()
curses.noecho()
curses.timeout(0)
curses.move(1, 0)
curses.printw('Hit <q> to stop logging...\n')
curses.refresh()

while (key ~= string.byte('q')) do
  -- update timing
  local t_elapsed = unix.time() - t
  if (delay > t_elapsed*1e6) then
    unix.usleep(delay - t_elapsed*1e6)
  end
  t = unix.time()

  -- log data
  local force = dcm:get_motor_force_sensor()
  for i = 1,#force do
    f:write(string.format('%f ', force[i]))
  end
  f:write(string.format('\n'))

  -- get user input
  curses.move(0, 0)
  curses.printw('%-7.4f seconds ', t - t0)
  key = curses.getch()
  if key == curses.KEY_RESIZE then
    local y, x = curses.getmaxyx()
    curses.resizeterm(y, x)
  end
end

curses.endwin()
f:close()
