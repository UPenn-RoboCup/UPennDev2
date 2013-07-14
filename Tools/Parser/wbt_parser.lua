
function check_comments(line, first_char)
  local first_char = first_char or 1
  -- find first and last #
  local first_shape, last_shape = line:find('#')
  if first_shape then
    -- trunk line in case some of it commented out
    if first_char < first_shape then
      return true, line:sub(1, first_shape - 1)
    end
    return false
  end
  return true, line
end

function check_blank(line)
  if line:len() == 0 then
    return false, 0
  else
    -- all space line
    local line_idx = 1
    while line_idx <= line:len() and line:byte(line_idx) == 32 do
      line_idx = line_idx + 1
    end
    if line_idx > line:len() then
      return false, 0
    else
      return true, line_idx
    end
  end
end

function read_wbt(wbt_file)
  local wbt = io.open(wbt_file, 'r')
  local wbt_lines = {}
  
  local line_counter = 0
  for line in wbt:lines() do
    local line_valid = true
    -- check if line is blank
    local valid_blank, first_valid_char = check_blank(line)
    line_valid = line_valid and valid_blank
    -- check if line has comments
    local valid_comments, valid_line = check_comments(line, first_valid_char)
    line_valid = line_valid and valid_comments
  
    if line_valid then
      wbt_lines[#wbt_lines + 1] = valid_line
      line_counter = line_counter + 1 
    end
  end
  print("lines ", #wbt_lines)
  
  wbt:close()
  return wbt_lines
end

function find_parenthesis(line)
  -- TODO multiple parenthesis on same line
  local p_left = {}
  local p_right = {}
  p_iter = line:gmatch('{')
  parenthesis = p_iter()
  while parenthesis do
    p_left[#p_left + 1] = parenthesis
    parenthesis = p_iter()
  end
  p_iter = line:gmatch('}')
  parenthesis = p_iter()
  while parenthesis do
    p_right[#p_right + 1] = parenthesis
    parenthesis = p_iter()
  end
  if (#p_left + #p_right) > 1 then
    error("only support one parenthesis per line")
  end
  if (#p_left > 0) then return p_left end
  if (#p_right > 0) then return p_right end
end

function find_sqbracket(line)
  -- TODO multiple sqbracket on same line
  local sqb_left = {}
  local sqb_right = {}
  sqb_iter = line:gmatch('%[')
  sqbracket = sqb_iter()
  while sqbracket do
    sqb_left[#sqb_left + 1] = sqbracket 
    sqbracket = sqb_iter()
  end
  sqb_iter = line:gmatch('%]')
  sqbracket = sqb_iter()
  while sqbracket do
    sqb_right[#sqb_right + 1] = sqbracket
    sqbracket = sqb_iter()
  end
  if (#sqb_left + #sqb_right) > 1 then
    error("only support one square bracket per line")
  end
  if (#sqb_left > 0) then return sqb_left end
  if (#sqb_right > 0) then return sqb_right end
end

function prune_spaces(line)
  -- trunk front and end spaces
  local idx = 1
  while line:byte(idx) == 32 do idx = idx + 1 end
  line = line:sub(idx, #line)
  local idx = line:len()
  while line:byte(idx) == 32 do idx = idx - 1 end
  line = line:sub(1, idx)
  return line
end

-- find children from 'children ['
function get_tag_str(line)
  local sqb_pos = line:find('%[')
  tag = line:sub(1, sqb_pos - 1)
  tag = prune_spaces(tag)
  return tag
end

-- find Solid from 'Solid {'
function get_field_str(line)
  local p_pos = line:find('{')
  field = line:sub(1, p_pos - 1)
  field = prune_spaces(field)
  return field
end

function get_line_str(line)
  return prune_spaces(line)
end

local field_name = ''

function construct_tree(lines, types, matches, lbegin, lend)
  local root = {}
  local idx = lbegin
  local field_name = ''
  while idx <= lend do
    if types[idx] == 'left_sqb' then
      local tag = get_tag_str(lines[idx])
      if #tag > 0 then field_name = tag end
      if not root[field_name] then root[field_name] = {} end
      root[field_name].attr = 
                construct_tree(lines, types, matches, idx + 1, matches[idx] - 1)
      idx = matches[idx] + 1
    elseif types[idx] == 'left_p' then
      local field = get_field_str(lines[idx])
      if #field > 0 then field_name = field end
      if not root[field_name] then root[field_name] = {} end
      root[field_name].sub = 
                construct_tree(lines, types, matches, idx + 1, matches[idx] - 1)
      idx = matches[idx] + 1
    else
      root[#root + 1] = get_line_str(lines[idx])
      idx = idx + 1
    end
  end
  return root
end

function check_bracket_match(lines)
  local line_type = {}
  local line_match = {}
  -- parse first line 
  local bracket_stack = {}
  local bracket_line_num_stack = {}
  for l = 1, #lines do
    local line = lines[l]
    -- check left bracket
    local parenthesis = find_parenthesis(line)
    local sqbracket = find_sqbracket(line)
    if parenthesis and sqbracket then
      error("only support one parenthesis or one square bracket per line")
    end
    if parenthesis then
      local p = parenthesis[1]
      if p == '{' then -- start field
        line_type[l] = 'left_p'
        local field_name_new = get_field_str(line)
        if #field_name_new > 0 then field_name = field_name_new end
--        print('field', field_name)
        bracket_stack[#bracket_stack + 1] = p
        bracket_line_num_stack[#bracket_line_num_stack + 1] = l
      else
        line_type[l] = 'right_p'
        line_match[l] = bracket_line_num_stack[#bracket_line_num_stack]
        line_match[bracket_line_num_stack[#bracket_line_num_stack]] = l
--        print('{}', bracket_line_num_stack[#bracket_line_num_stack], l)
        if bracket_stack[#bracket_stack] ~= '{' then
          error(string.format('right parenthesis not match left on line %d', l))
        end
        -- find match and remove
        table.remove(bracket_stack, #bracket_stack)
        table.remove(bracket_line_num_stack, #bracket_line_num_stack)
      end
    end
    if sqbracket then
      local sqb = sqbracket[1]
      if sqb == '[' then -- start attributes
        line_type[l] = 'left_sqb'
        local tag_name_new = get_tag_str(line)
        if #tag_name_new > 0 then field_name = tag_name_new end
--        print('tag', field_name)
        bracket_stack[#bracket_stack + 1] = sqb
        bracket_line_num_stack[#bracket_line_num_stack + 1] = l
      else
        line_type[l] = 'right_sqb'
        line_match[l] = bracket_line_num_stack[#bracket_line_num_stack]
        line_match[bracket_line_num_stack[#bracket_line_num_stack]] = l
--        print('[]', bracket_line_num_stack[#bracket_line_num_stack], l)
        if bracket_stack[#bracket_stack] ~= '[' then
          error(string.format('right square brackets not match left on line %d', l))
        end
        -- find match and remove
        table.remove(bracket_stack, #bracket_stack)
        table.remove(bracket_line_num_stack, #bracket_line_num_stack)
      end
    end
  end
  return line_type, line_match
end

function parse_number(lines, deli)
  local nums = {}
  for i = 1, #lines do
    if deli == ',' then
      num_iter = lines[i]:gmatch("([^,]+),?");
    elseif deli == ' ' then
      num_iter = lines[i]:gmatch("%S+");
    end
    num = num_iter()
    while num do
      nums[#nums + 1] = num
--      print(tonumber(num))
      num = num_iter()
    end
  end
  for i = 1, #nums do
    io.write(string.format('%4.3f ', nums[i]))
  end
  io.write('\n')
end

function print_tree(root, indent)
  local INDENT = function(indent)
    local str = ''
    for i = 1, indent do
      str = str..' '
    end
    return str
  end
  for k, v in pairs(root) do
    if k == 'attr' then
      print_tree(root.attr, indent + 1)
    elseif k == 'sub' then
      print_tree(root.sub, indent + 1)
    elseif type(k) == 'number' then
      print(INDENT(indent)..v)
    elseif k == 'coordIndex' or k == 'texCoordIndex' then
      parse_number(root[k].attr, ',') 
    elseif k == 'point' then
      parse_number(root[k].attr, ' ')
    else
      print(INDENT(indent)..k)
      print_tree(root[k], indent + 1)
    end
  end
end

local file_name = '../../WebotsProj/protos/TeenWalls.proto'
wbt_lines = read_wbt(file_name)
-- root.string : raw field_name
-- root.attr 
-- root[1] root[2] 
line_type, line_match = check_bracket_match(wbt_lines)
-- print(#line_type, #line_match)
-- for k, v in pairs(line_match) do
--   if line_type[k]:find('left') then
--     print(wbt_lines[k], line_type[k], k, v)
--   end
-- end
root = construct_tree(wbt_lines, line_type, line_match, 1, #wbt_lines)
print_tree(root, 0)
