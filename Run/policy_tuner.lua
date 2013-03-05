dofile('include.lua')

--------------------------------------------------------------------------------
-- Policy Tuner
--------------------------------------------------------------------------------

require('rpc')
require('unix')
require('util')
require('getch')
require('curses')

-- connect to motion manager
local context = zmq.init()
local motion_manager_endpoint = 'tcp://localhost:12000'
local motion_manager = rpc.client.new(motion_manager_endpoint, context)
print('Attempting to connect to motion_manager at '..motion_manager_endpoint)
motion_manager:connect(nil)
motion_manager:set_timeout(0.05)

local state = 'stand'
local parameter_keys = {}
local parameter_values = {}

local TIMEOUT = 1
local NCOLS = 1
local NROWS = #parameter_keys
local ROW_WIDTH = 1
local COL_WIDTH = 14
local COL_OFFSET = 30
local ROW_OFFSET = 4
local COL_PARAM = 1
local ROW_CMD = NROWS + 2

local row = 1
local col = 1

-- Commands
--------------------------------------------------------------------------------

function cmd_set_value(arg)
  local varg = parse_double_arguments(arg)
  if varg[1] then
    set_value(varg[1], row, col)
  end
  draw_screen()
end

function cmd_increment_value(scale)
  local val = get_value(row, col)
  if val then
    val = val + scale*get_increment(row, col)  
    set_value(val, row, col)
  end
  draw_col(COL_PARAM)
end

function cmd_set_state(arg)
  -- FIXME check for valid motion states
  local varg = parse_string_arguments(arg)
  state = varg[1]
  draw_screen()
end

function cmd_save(arg)
  -- FIXME implement remote parameter saving
  local varg = parse_string_arguments(arg)
  local parameter_file = nil
  if (varg[1]) then
    parameter_file = '../Data/'..Config.platform.name..'/'..varg[1]..'.lua'
  end
  motion_manager:call(state..':save_parameters', parameter_file)
end

function cmd_quit()
  exit()
  os.exit()
end

function cmd_help()
  curses.timeout(-1)
  curses.clear()
  curses.move(0, 0)
  curses.printw('-----------------------------------------------------------\n')
  curses.printw('edit [state]          select motion state to edit\n')
  curses.printw('set [value]           set value under cursor\n')
  curses.printw('save [filename]       write current parameters to file\n')
  curses.printw('q                     quit\n')
  curses.printw('h                     help\n')
  curses.printw('press any key to continue...')
  curses.getch()
  curses.clear()
  draw_screen()
  curses.timeout(TIMEOUT)
end

local commands = {
  ['edit'] = cmd_set_state,
  ['set'] = cmd_set_value,
  ['save'] = cmd_save,
  ['h'] = cmd_help,
  ['help'] = cmd_help,
  ['q'] = cmd_quit,
  ['quit'] = cmd_quit,
  ['exit'] = cmd_quit,
}

-- Access
-------------------------------------------------------------------------------

function update_parameters()
  local accessor = state..':get_parameters'
  while (not motion_manager:call(accessor)) do 
  end
  parameter_values = motion_manager:get_return_values()
  parameter_keys = {}
  for k,v in pairs(parameter_values) do
    parameter_keys[#parameter_keys + 1] = k
  end
  NROWS = #parameter_keys
  ROW_CMD = NROWS + 2
end

function get_value(r, c)
  if (c == COL_PARAM) then
    return parameter_values[parameter_keys[r]]
  end
end

function set_value(val, r, c)
  if (c == COL_PARAM) then
    local accessor = state..':set_parameter'
    while (not motion_manager:call(accessor, parameter_keys[r], val)) do
    end
    parameter_values[parameter_keys[r]] = val
  end
end

function get_increment(r, c)
  return 0.001 
end

-- Display
--------------------------------------------------------------------------------

function draw_command_row(str)
  local cursory = get_cursor(ROW_CMD, 0)
  curses.move(cursory, 0)
  curses.printw(':%80s', ' ')
  curses.move(cursory, 2)
  curses.printw(str)
  curses.refresh()
end

function draw_col(c)
  for r = 1,NROWS do
    curses.move(get_cursor(r, c))
    val = get_value(r, c)
    if val then
      curses.printw('[%10.4f]', val)
    else
      curses.printw('[----------]')
    end
  end
  curses.move(get_cursor(row, col))
  curses.refresh()
end

function draw_screen()
  update_parameters()
  curses.clear()
  curses.move(0, 0)
  curses.printw('                                Policy Tuner\n')
  curses.printw('///////////////////////////////////////')
  curses.printw('//////////////////////////////////////\n')
  curses.printw(' %-16s %10s %12s \n', state, 'parameter', 'value')
  curses.printw('---------------------------------------')
  curses.printw('--------------------------------------\n')
  for i = 1,NROWS do
    curses.printw('%2d', i)
    curses.printw(' %25s\n', parameter_keys[i])
  end
  for c = 1,NCOLS do
    draw_col(c)
  end
  curses.refresh()
  curses.move(get_cursor(row, col))
end

-- Parsing
--------------------------------------------------------------------------------

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
  curses.timeout(TIMEOUT)
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

-- Navigation
--------------------------------------------------------------------------------

function get_cursor(r, c)
  local y = ROW_OFFSET + (r-1)*ROW_WIDTH
  local x = COL_OFFSET + (c-1)*COL_WIDTH
  return y, x 
end

function cursor_right()
  col = math.min(col + 1, COL_PARAM)
  curses.move(get_cursor(row, col))
end

function cursor_left()
  col = math.max(col - 1, COL_PARAM)
  curses.move(get_cursor(row, col))
end

function cursor_up()
  row = math.max(row - 1, 1)
  curses.move(get_cursor(row, col))
end

function cursor_down()
  row = math.min(row + 1, NROWS)
  curses.move(get_cursor(row, col))
end

-- Main
--------------------------------------------------------------------------------

function entry()
  -- setup curses environment
  curses.initscr()
  curses.cbreak()
  curses.noecho()
  curses.keypad(1)
  curses.timeout(TIMEOUT)
  draw_screen()
  -- initialize shared memory
  unix.usleep(5e5)
  draw_screen()
end

function update()
  -- handle keystrokes
  local key = curses.getch()
  if key == string.byte('-') then
    cmd_increment_value(-0.1)
  elseif key == string.byte('=') then
    cmd_increment_value(0.1)
  elseif key == string.byte('_') then
    cmd_increment_value(-0.5)
  elseif key == string.byte('+') then
    cmd_increment_value(0.5)
  elseif key == string.byte('[') then
    cmd_increment_value(-1)
  elseif key == string.byte(']') then
    cmd_increment_value(1)
  elseif key == string.byte('{') then
    cmd_increment_value(-10)
  elseif key == string.byte('}') then
    cmd_increment_value(10)
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
end

function exit()
  motion_manager:close()
  context:term()
  curses.endwin()
end

entry()
while true do
  update()
end
exit()
