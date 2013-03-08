dofile('include.lua')

----------------------------------------------------------------------
-- Motion Manager
----------------------------------------------------------------------

require('zmq')
require('rpc')
require('unix')
require('util')
require('Platform')
require('curses')
require('Config')
require('Motion')
require('Attention')
require('Locomotion')
require('Manipulation')
require('Proprioception')

local rpc_endpoint = 'tcp://127.0.0.1:12001'

local function draw_screen()
  curses.clear()
  curses.printw('fps : %7.2f                   Motion Manager\n', 
                 Platform.get_update_rate())
  curses.printw('///////////////////////////////////////')
  curses.printw('///////////////////////////////////////\n')
  curses.printw('               %25s %25s\n', 'state', 'event')
  curses.printw('---------------------------------------')
  curses.printw('---------------------------------------\n')
  curses.printw('Locomotion   : %25s %25s\n',
                 Locomotion:get_state(), Locomotion:get_event() or '')
  curses.printw('Manipulation : %25s %25s\n',
                 Manipulation:get_state(), Manipulation:get_event() or '')
  curses.printw('Attention    : %25s %25s\n',
                 Attention:get_state(), Attention:get_event() or '')
  curses.refresh()
end

-- initialize motion state machines
Platform.entry()
Proprioception.entry()
Motion.entry()
Motion.add_fsm(Attention)
Motion.add_fsm(Locomotion)
Motion.add_fsm(Manipulation)
Locomotion:add_event('stand')

-- initialize screen
curses.initscr()
curses.cbreak()
curses.noecho()
curses.keypad(1)
curses.timeout(1)
draw_screen()

local count = 0
local context = zmq.init()
local motion_rpc_server = rpc.server.new(rpc_endpoint, context)
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
    draw_screen()
  end
end

motion_rpc_server:close()
context:term()
curses.endwin()

Motion.exit()
Proprioception.exit()
Platform.exit()
