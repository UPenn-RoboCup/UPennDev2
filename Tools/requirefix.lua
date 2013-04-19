function require_fix(filename)
  local file = io.open(filename, 'r')
  local new_file_str = ''
  for line in file:lines() do
    if line:find('require') then
      print(line)
      local first_quota = line:find('\'')
      if first_quota then
        local second_quota = line:find('\'', first_quota + 1)
        local module_name = line:sub(first_quota+1, second_quota-1)
        local first_module_name = line:find(module_name)
        if first_module_name == (first_quota + 1) then
          print('need fix')
          local new_line = line:gsub('require', 'local '..module_name..' = require') 
          line = new_line
          print(new_line)
        end
        print(first_quota, second_quota, module_name, first_module_name)
      end
    end
    new_file_str = new_file_str..line..'\n';
  end
  local new_file = io.open(filename, 'w+')
  new_file:write(new_file_str)
  new_file:close()
  file:close()
end

for i = 1, #arg do
  print(arg[i])
  require_fix(arg[i])
end
