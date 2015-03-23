local function update(key_code)
  -- Assure that we got a character
  if type(key_code)~='number' or key_code==0 then return end
	local key_char = string.char(key_code)
	local key_char_lower = string.lower(key_char)
  
  local code_f, char_f, lower_f = code_lut[key_code], char_lut[key_char], lower_lut[key_char_lower]
  print('Keycodes', key_code,key_char,key_char_lower)
  print('Functions',code_f, char_f, lower_f)
  -- Precedence
  if type(code_f)=='function' then
    code_f()
  elseif type(char_f)=='function' then
    char_f()
  elseif type(lower_f)=='function' then
    lower_f()
  else
    io.write('\nkey_code:', key_code, 'key_char:', key_char)
  end
	if type(show_status)=='function' then
		if not IS_WEBOTS then os.execute('clear') end
		show_status()
	end
end

-- Show the initial status
if type(show_status)=='function' then
	if not IS_WEBOTS then os.execute('clear') end
	show_status()
end

-- Loop if not webots
local getch = require'getch'
local running = not IS_WEBOTS
local key_code
while running do
	key_code = getch.block()
  update(key_code)
end

return {entry=nil, update=update, exit=nil}
