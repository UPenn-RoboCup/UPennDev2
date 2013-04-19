dofile('include.lua')

----------------------------------------------------------------------
-- Motion Manager
----------------------------------------------------------------------

require('zmq')
require('rpc')
require('unix')
require('util')
require('curses')
require('Config')
require('Motion')
require('Platform')
require('Proprioception')

local rpc_endpoint = 'tcp://127.0.0.1:12001'

Platform.set_update_rate(500)

Platform.entry()
Proprioception.entry()
Motion.entry()

-- initialize motion state machines
local motion_fsm_names = Config.motion.fsms or
{
  'Locomotion', 'Manipulation', 'Attention'
}

local motion_fsms = {}
for i = 1,#motion_fsm_names do
  motion_fsms[i] = require(motion_fsm_names[i])
end

for i = 1,#motion_fsms do
  Motion.add_fsm(motion_fsms[i])
end

-- initialize screen
local function draw_screen()
  curses.clear()
  curses.printw('rate : %7.2f                  Motion Manager\n', 
                 Platform.get_update_rate())
  curses.printw('///////////////////////////////////////')
  curses.printw('///////////////////////////////////////\n')
  curses.printw('               %25s %25s\n', 'state', 'event')
  curses.printw('---------------------------------------')
  curses.printw('---------------------------------------\n')
  for i = 1,#motion_fsms do
    curses.printw('%-12s : %25s %25s\n', motion_fsm_names[i],
      motion_fsms[i]:get_state(), motion_fsms[i]:get_event() or '')
  end
  curses.refresh()
end

curses.initscr()
curses.cbreak()
curses.noecho()
curses.keypad(1)
curses.timeout(1)
draw_screen()

local count = 0
local motion_rpc_server = rpc.server.new(rpc_endpoint)
motion_rpc_server:set_timeout(0)

while true do
  -- handle remote procedure calls
  motion_rpc_server:update()

  -- update controllers
  Platform.update()
  Proprioception.update()
  Motion.update()

  -- handle keystrokes
  local key = curses.getch()
  if (key == string.byte('q')) then
    break
  end

  -- update screen
  count = count + 1
  if (key or (count % 20 == 0)) then
    if key == curses.KEY_RESIZE then
      local y, x = curses.getmaxyx()
      curses.resizeterm(y, x)
    end
    draw_screen()
  end
end

motion_rpc_server:close()
curses.endwin()

Motion.exit()
Proprioception.exit()
Platform.exit()
