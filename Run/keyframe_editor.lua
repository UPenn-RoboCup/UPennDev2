dofile('include.lua')

----------------------------------------------------------------------
-- Keyframe Editor
----------------------------------------------------------------------

require('acm')
require('scm')
require('Body')
require('util')
require('unix')
require('curses')
require('Config')
require('keyframe')
require('serialization')

local action_table_name = arg[1] or Config.platform.action_table
pcall(require, action_table_name)

local joint = Config.joint
local page_no = 1
local step_no = 1
local command_duration = 1 
local command_pause = 0

local NCOLS = 6 
local NROWS = #joint.id + 3
local ROW_WIDTH = 1
local COL_WIDTH = 9
local COL_OFFSET = 22
local ROW_OFFSET = 4
local COL_CMD = 1
local COL_STEP = 2
local ROW_JOINT = 1
local ROW_LINE = NROWS - 2
local ROW_DURAT = NROWS - 1 
local ROW_PAUSE = NROWS
local ROW_CMD = NROWS + 1

local row = 1 
local col = 1

-- Data
----------------------------------------------------------------------

function new_page()
  return {name = '', steps = {}}
end

function create_action_table()
  local pages = {}
  for i = 1,99 do
    pages[i] = new_page()
  end
  return pages
end

function save_action_table()
  local serialize = serialization.serialize
  local f = assert(io.open('../Data/'..action_table_name..'.lua','w+'))
  f:write('action_table = {}\n')
  for i = 1,#action_table do
    local page = action_table[i]
    f:write(string.format('action_table[%d] = {\n', i))
    f:write(string.format('  name = "%s",\n', page.name))
    f:write(string.format('  steps = {\n'))
    for j = 1,#page.steps do
      local step = page.steps[j]
      f:write(string.format('    {\n'))
      f:write(string.format('      joint_position = %s, \n',
        serialize(step.joint_position)))
      f:write(string.format('      duration = %f,\n', step.duration))
      f:write(string.format('      pause = %f,\n', step.pause))
      f:write(string.format('    },\n')) 
    end
    f:write(string.format('  }\n')) 
    f:write(string.format('}\n')) 
  end
  f:write('return action_table')
  f:close()
end

-- Commands
----------------------------------------------------------------------

function cmd_on(arg)
  local joints = {}
  local id = parse_string_arguments(arg)[1]
  if (id and joint.index[id]) then
    joints = joint.index[id]
  else
    for i,index in pairs(parse_int_arguments(arg)) do
      if (index > 0) and (index <= #joint.id) then
         joints[#joints+1] = index
      end
    end
  end
  acm:set_joint_position(scm:get_joint_position(joints), joints)
  acm:set_joint_enable(1, joints)
  draw_screen()
end

function cmd_off(arg)
  local joints = {}
  local id = parse_string_arguments(arg)[1]
  if (id and joint.index[id]) then
    joints = joint.index[id]
  else
    for i,index in pairs(parse_int_arguments(arg)) do
      if (index > 0) and (index <= #joint.id) then
         joints[#joints+1] = index
      end
    end
  end
  acm:set_joint_enable(0, joints)
  draw_screen()
end

function cmd_set_value(arg)
  local varg = parse_double_arguments(arg)
  if varg[1] then
    set_value(varg[1], row, col)
  end
  draw_col(col)
end

function cmd_increment_value(scale)
  val = get_value(row, col)
  if val then
    val = val + scale*get_increment(row, col)  
    set_value(val, row, col)
  end
  draw_col(col)
end

function cmd_write_step(arg)
  local varg = parse_int_arguments(arg)
  local step_index = varg[1]
  if (step_index and step_index > 0) then
    action_table[page_no].steps[step_index] = { 
      joint_position = acm:get_joint_position(),
      duration = command_duration,
      pause = command_pause,
    }
  else
    draw_command_row('Invalid step index')
  end
  draw_screen()
end

function cmd_insert_step(arg)
  local varg = parse_int_arguments(arg)
  local step_index = varg[1]
  if (step_index and step_index > 0) then
    table.insert(action_table[page_no].steps, step_index, { 
      joint_position = acm:get_joint_position(),
      duration = command_duration,
      pause = command_pause,
    })
  else
    draw_command_row('Invalid step index')
  end
  draw_screen()
end

function cmd_copy_step(arg)
  local varg = parse_int_arguments(arg)
  local step_index = varg[2]
  local source = varg[1] and action_table[page_no].steps[varg[1]]
  if (step_index and step_index > 0) and source then
    action_table[page_no].steps[step_index] = {
      joint_position = vector.copy(source.joint_position),
      duration = source.duration,
      pause = source.pause,
    }
  else
    draw_command_row('Invalid step index')
  end
  draw_screen()
end

function cmd_delete_step(arg)
  local varg = parse_int_arguments(arg)
  local step_index = varg[1]
  if (step_index and step_index > 0) then
    if (step_index > #action_table[page_no].steps) then
      action_table[page_no].steps[step_index] = nil
    else
      table.remove(action_table[page_no].steps, step_index)
    end
  else
    draw_command_row('Invalid step index')
  end
  draw_screen()
end

function cmd_previous_step()
  step_no = math.max(step_no - 1, 1)
  draw_screen()
end

function cmd_next_step()
  step_no = math.min(step_no + 1, 95)
  draw_screen()
end

function cmd_name_page(arg)
  local varg = parse_string_arguments(arg)
  action_table[page_no].name = varg[1] or ''
  draw_screen()
end

function cmd_clear_page()
  action_table[page_no] = new_page()
  draw_screen()
end

function cmd_read_page(arg)
  local varg = parse_int_arguments(arg)
  local page = varg[1] and action_table[varg[1]]
  local offset = table.maxn(action_table[page_no].steps)
  if page then
    for i = 1,#page.steps do
      local step = page.steps[i]
      action_table[page_no].steps[offset + i] = {
        joint_position = vector.copy(step.joint_position),
        duration = step.duration,
        pause = step.pause,
      }
    end
  end
  draw_screen()
end

function cmd_page(arg)
  local varg = parse_int_arguments(arg)
  if (varg[1] and varg[1] > 0 and varg[1] <= #action_table) then
    page_no = varg[1]
  else
    draw_command_row('Invalid page index')
  end
  draw_screen()
end

function cmd_next_page()
  step_no = 1
  page_no = math.min(page_no + 1, #action_table)
  draw_screen()
end

function cmd_previous_page()
  step_no = 1
  page_no = math.max(page_no - 1, 1)
  draw_screen()
end

function cmd_goto(arg)
  local varg = parse_int_arguments(arg)
  local step = varg[1] and action_table[page_no].steps[varg[1]]
  if step then
    local page = new_page()
    page.steps[1] = {
      joint_position = step.joint_position,
      duration = 3,
      pause = 0,
    }
    keyframe:stop()
    keyframe:play(page)
    keyframe:entry()
    draw_command_row('going...')
    while true do
      if curses.getch() or (keyframe:update() == 'done') then
        keyframe:stop()
        break
      end
      Body.update()
      draw_col(COL_CMD)
    end
    keyframe:exit()
    draw_command_row('done')
  end
end

function cmd_zero()
  local page = new_page()
  page.steps[1] = {
    joint_position = vector.zeros(#joint.id),
    duration = 3,
    pause = 0,
  }
  keyframe:stop()
  keyframe:play(page)
  keyframe:entry()
  draw_command_row('going...')
  while true do
    if curses.getch() or (keyframe:update() == 'done') then
      keyframe:stop()
      break
    end
    Body.update()
    draw_col(COL_CMD)
  end
  keyframe:exit()
  draw_command_row('done')
end

function cmd_play()
  keyframe:load_action_table(action_table)
  keyframe:stop()
  keyframe:play(page_no)
  keyframe:entry()
  draw_command_row('playing...')
  while true do
    if curses.getch() or (keyframe:update() == 'done') then
      keyframe:stop()
      break
    end
    Body.update()
    draw_col(COL_CMD)
  end
  keyframe:exit()
  draw_command_row('done')
end

function cmd_loop(arg)
  local count = parse_int_arguments(arg)[1] or 0
  keyframe:load_action_table(action_table)
  for i = 1,count do
    keyframe:play(page_no)
  end
  keyframe:entry()
  draw_command_row('playing...')
  while true do
    if curses.getch() or (keyframe:update() == 'done') then
      keyframe:stop()
      break
    end
    Body.update()
    draw_col(COL_CMD)
  end
  keyframe:exit()
  draw_command_row('done')
end

function cmd_save(arg)
  local varg = parse_string_arguments(arg)
  action_table_name = varg[1] or action_table_name 
  save_action_table()
end

function cmd_quit()
  exit()
  os.exit()
end

function cmd_help()
  curses.timeout(-1)
  curses.clear()
  curses.move(0, 0)
  curses.printw('------------------------------------------------ Navigation\n')
  curses.printw('save           write current action_table to file\n')
  curses.printw('page [i]       move to page i\n')
  curses.printw('n              next page\n')
  curses.printw('p              previous page\n')
  curses.printw('q              quit\n')
  curses.printw('h              help\n')
  curses.printw('------------------------------------------------ Editing\n')
  curses.printw('clear          reset current page\n')
  curses.printw('name [string]  set page name\n')
  curses.printw('set [value]    set value under the cursor\n')
  curses.printw('r [i]          copy steps from page i\n')
  curses.printw('w [i]          write current commands to step i\n')
  curses.printw('i [i]          insert current commands at step i\n')
  curses.printw('c [i] [j]      copy step i to step j\n')
  curses.printw('d [i]          delete step i\n')
  curses.printw('------------------------------------------------ Motion\n')
  curses.printw('zero           move to zero pose \n')
  curses.printw('goto [i]       move to step i\n')
  curses.printw('play           play current page \n')
  curses.printw('loop [count]   loop current page\n')
  curses.printw('on  [i] [j]... enable joints i, j, ...\n')
  curses.printw('off [i] [j]... disable joints i, j, ...\n\n')
  curses.printw('press any key to continue...')
  curses.getch()
  curses.clear()
  draw_screen()
  curses.timeout(10)
end

local commands = {
  ['save'] = cmd_save,
  ['page'] = cmd_page,
  ['name'] = cmd_name_page,
  ['clear'] = cmd_clear_page,
  ['set'] = cmd_set_value,
  ['on'] = cmd_on,
  ['off'] = cmd_off,
  ['zero'] = cmd_zero,
  ['goto'] = cmd_goto,
  ['play'] = cmd_play,
  ['loop'] = cmd_loop,
  ['r'] = cmd_read_page,
  ['read'] = cmd_read_page,
  ['w'] = cmd_write_step,
  ['write'] = cmd_write_step,
  ['i'] = cmd_insert_step,
  ['insert'] = cmd_insert_step,
  ['c'] = cmd_copy_step,
  ['copy'] = cmd_copy_step,
  ['d'] = cmd_delete_step,
  ['delete'] = cmd_delete_step,
  ['n'] = cmd_next_page,
  ['next'] = cmd_next_page,
  ['p'] = cmd_previous_page,
  ['previous'] = cmd_previous_page,
  ['h'] = cmd_help,
  ['help'] = cmd_help,
  ['q'] = cmd_quit,
  ['quit'] = cmd_quit,
  ['exit'] = cmd_quit,
}

-- Parsing
-----------------------------------------------------------------

function parse_int_arguments(arg)
  local varg = {}
  for int in arg:gmatch('[ ,](%-?%d+)') do
    varg[#varg + 1] = tonumber(int)
  end
  return varg
end

function parse_double_arguments(arg)
  local varg = {}
  for double in arg:gmatch('[ ,](%-?%d*%.?%d+e?-?%d*)') do
    varg[#varg + 1] = tonumber(double)
  end
  return varg
end

function parse_string_arguments(arg)
  local varg = {}
  for token in arg:gmatch('[ ,](%a[%a_%d]*)') do
    varg[#varg + 1] = token 
  end
  return varg
end

function read_command(key)
  -- clear prompt
  draw_command_row('')
  -- get command string
  curses.echo()
  curses.timeout(-1)
  local str = curses.getstr()
  curses.timeout(10)
  curses.noecho()
  -- call command
  local cmd, arg = str:match('^([%a_]+)(.*)$')
  if commands[cmd] then
    commands[cmd](arg)
  elseif cmd then
    draw_command_row('Invalid command')
  end
  -- restore cursor 
  curses.move(get_cursor(row, col))
end

-- Access
---------------------------------------------------------------------

function get_value(r, c)
  -- command column
  if (c == COL_CMD) then
    if (r < ROW_LINE) then
      local id = 1 + (r - ROW_JOINT)
      if (acm:get_joint_enable(id) == 1) then
        return acm:get_joint_position(id)
      elseif (acm:get_joint_enable(id) == 0) then
        return scm:get_joint_position(id)
      end
    elseif (r == ROW_DURAT) then
      return command_duration
    elseif (r == ROW_PAUSE) then
      return command_pause
    end
  -- step column
  elseif (c >= COL_STEP) then
    local step = step_no + (c - COL_STEP)
    if (not action_table[page_no].steps[step]) then
      return
    end
    if (r < ROW_LINE) then
      local id = 1 + (r - ROW_JOINT)
      return action_table[page_no].steps[step].joint_position[id]
    elseif (r == ROW_DURAT) then
      return action_table[page_no].steps[step].duration
    elseif (r == ROW_PAUSE) then
      return action_table[page_no].steps[step].pause
    end
  end
end

function set_value(val, r, c)
  -- command column
  if (c == COL_CMD) then
    if (r < ROW_LINE) then
      local id = 1 + (r - ROW_JOINT)
      acm:set_joint_position(val, id)
    elseif (r == ROW_DURAT) then
      command_duration = val
    elseif (r == ROW_PAUSE) then
      command_pause = val
    end
  -- step column
  elseif (c >= COL_STEP) then
    local step = step_no + (col - COL_STEP)
    if (not action_table[page_no].steps[step]) then
      return
    end
    if (r < ROW_LINE) then
      local id = 1 + (r - ROW_JOINT)
      action_table[page_no].steps[step].joint_position[id] = val
    elseif (r == ROW_DURAT) then
      action_table[page_no].steps[step].duration = val
    elseif (r == ROW_PAUSE) then
      action_table[page_no].steps[step].pause = val
    end
  end
end

function get_increment(r, c)
  if (r < ROW_LINE) then
    return 0.01
  elseif (r == ROW_DURAT) then
    return 0.05
  elseif (r == ROW_PAUSE) then
    return 0.05
  end
end

-- Navigation
----------------------------------------------------------------------

function get_cursor(r, c)
  local y = ROW_OFFSET + (r-1)*ROW_WIDTH
  local x = COL_OFFSET + (c-1)*COL_WIDTH
  return y, x 
end

function cursor_right()
  if (col == NCOLS) then
    cmd_next_step()
  end
  col = math.min(col + 1, NCOLS)
  curses.move(get_cursor(row, col))
end

function cursor_left()
  if (col == 1) then
    cmd_previous_step()
  end
  col = math.max(col - 1, 1)
  curses.move(get_cursor(row, col))
end

function cursor_up()
  row = math.max(row - 1, 1)
  if (row == ROW_LINE) then
    row = row - 1
  end
  curses.move(get_cursor(row, col))
end

function cursor_down()
  row = math.min(row + 1, NROWS)
  if (row == ROW_LINE) then
    row = row + 1
  end
  curses.move(get_cursor(row, col))
end

-- Display
----------------------------------------------------------------------

function draw_command_row(str)
  local cursory = get_cursor(ROW_CMD, 0)
  curses.move(cursory, 0)
  curses.printw(':%80s', ' ')
  curses.move(cursory, 2)
  curses.printw(str)
  curses.refresh()
end

function draw_col(c)
  for r = 1, NROWS do
    curses.move(get_cursor(r, c))
    val = get_value(r, c)
    if val then
      curses.printw('%6.3f', val)
    else
      curses.printw('------')
    end
  end
  curses.move(get_cursor(row, col))
  curses.refresh()
end

function draw_screen()
  curses.move(0, 0)
  curses.printw('                               Keyframe Editor\n')
  curses.printw('///////////////////////////////////////')
  curses.printw('///////////////////////////////////////\n')
  curses.printw('%2d %16s     cmd   ', page_no, action_table[page_no].name)
  for i = 0,4 do
    curses.printw('  stp%2d  ', step_no + i)
  end
  curses.printw('\n--------------------------------------')
  curses.printw('----------------------------------------\n')
  curses.refresh()
  unix.usleep(1000)
  for i = 1,#joint.id do
    if (acm:get_joint_enable(i) == 1) then
      curses.attron(curses.A_STANDOUT)
    end
    curses.printw('%2d', i)
    curses.attroff(curses.A_STANDOUT)
    curses.printw(' %16s\n', joint.id[i])
  end
  curses.printw('---------------------------------------')
  curses.printw('---------------------------------------\n')
  curses.printw('           duration\n')
  curses.printw('              pause\n')
  for c = 1,NCOLS do
    draw_col(c)
  end
  curses.move(get_cursor(row, col))
end

-- Main
------------------------------------------------------------------

function entry()
  -- setup curses environment
  curses.initscr()
  curses.cbreak()
  curses.noecho()
  curses.keypad(1)
  curses.timeout(10)
  -- initialize action table
  if (not action_table) then
    action_table = create_action_table()
  end
  draw_screen()
  -- initialize shared memory
  Body.entry()
  unix.usleep(5e5)
  acm:set_joint_position(scm:get_joint_position())
  draw_screen()
end

function update()
  -- handle keystrokes
  local key = curses.getch()
  if key == string.byte('-') then
    cmd_increment_value(-0.2)
  elseif key == string.byte('=') then
    cmd_increment_value(0.2)
  elseif key == string.byte('_') then
    cmd_increment_value(-1)
  elseif key == string.byte('+') then
    cmd_increment_value(1)
  elseif key == string.byte('[') then
    cmd_increment_value(-2)
  elseif key == string.byte(']') then
    cmd_increment_value(2)
  elseif key == string.byte('{') then
    cmd_increment_value(-5)
  elseif key == string.byte('}') then
    cmd_increment_value(5)
  elseif key == curses.KEY_UP then
    cursor_up()
  elseif key == curses.KEY_DOWN then
    cursor_down()
  elseif key == curses.KEY_LEFT then
    cursor_left()
  elseif key == curses.KEY_RIGHT then
    cursor_right()
  elseif key then
    curses.ungetch(key)
    read_command()
  end
  Body.update()
end

function exit()
  Body.exit()
  curses.endwin()
end

local stats = util.loop_stats(25)

entry()
while true do
  update()
  if stats.update() then
    draw_col(COL_CMD)
  end
end
exit()
