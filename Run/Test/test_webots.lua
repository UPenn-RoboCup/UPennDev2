dofile('include.lua')

----------------------------------------------------------------------
-- Test Webots Controller
----------------------------------------------------------------------

require('unix')
require('util')
require('Body')
require('curses')
require('Config')

Body.entry()

local stats = util.loopstats(20)

function draw_screen()
  curses.clear()
  curses.printw("actuator joint mode      [ %10.3f ]\n", acm:get_joint_mode(1));
  curses.printw("actuator joint enable    [ %10.3f ]\n", acm:get_joint_enable(1));
  curses.printw("actuator joint mode      [ %10.3f ]\n", acm:get_joint_mode(1));
  curses.printw("actuator joint position  [ %10.3f ]\n", acm:get_joint_position(1));
  curses.printw("actuator joint force     [ %10.3f ]\n", acm:get_joint_force(1));
  curses.printw("actuator joint stiffness [ %10.3f ]\n", acm:get_joint_stiffness(1));
  curses.printw("actuator joint damping   [ %10.3f ]\n\n", acm:get_joint_damping(1));

  curses.printw("sensor joint position    [ %10.3f ]\n", scm:get_joint_position(1));
  curses.printw("sensor joint force       [ %10.3f ]\n", scm:get_joint_force(1));
  curses.printw("sensor motor position    [ %10.3f ]\n", scm:get_motor_position(1));
  curses.printw("sensor motor force       [ %10.3f ]\n", scm:get_motor_force(1));
  curses.printw("sensor motor current     [ %10.3f ]\n", scm:get_motor_current(1));
  curses.printw("sensor motor temperature [ %10.3f ]\n\n", scm:get_motor_temperature(1));

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
  if (curses.getch() == string.byte('q')) then
    break
  end
  if (stats.update()) then
    draw_screen()
  end
end

curses.endwin()
Body.exit()
