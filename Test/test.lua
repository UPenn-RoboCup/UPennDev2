local targetvel = {0,0,0}
local targetvel_new = {0,0,0}
local WAS_REQUIRED

local t_last = Body.get_time()
local tDelay = 0.005*1E6
local command1, command2 = '',''



local function update(key_code)
  -- Assure that we got a character
  if type(key_code)~='number' or key_code==0 then return end
	local key_char = string.char(key_code)
	local key_char_lower = string.lower(key_char)
  
  local code_f, char_f, lower_f = code_lut[key_code], char_lut[key_char], lower_lut[key_char_lower]
  --print('Keycodes', key_code,key_char,key_char_lower)
  --print('Functions',code_f, char_f, lower_f)
	--io.write('\nkey_code:', key_code, 'key_char:', key_char,'\n')
  -- Precedence
  if type(code_f)=='function' then
    code_f()
  elseif type(char_f)=='function' then
    char_f()
  elseif type(lower_f)=='function' then
    lower_f()
  end
	if type(show_status)=='function' then
		if not IS_WEBOTS then os.execute('clear') end
		show_status()
	end

  ----------------------------------------------------------
  local lleft = hcm.get_legdebug_left()
  local lright = hcm.get_legdebug_right()
  local ltorso = hcm.get_legdebug_torso()


  local torsoangle = hcm.get_legdebug_torso_angle()
  if key_char_lower==("i") then    targetvel_new[1]=targetvel[1]+0.05;
  elseif key_char_lower==("j") then targetvel_new[3]=targetvel[3]+0.15;
  elseif key_char_lower==("k") then targetvel_new[1],targetvel_new[2],targetvel_new[3]=0,0,0;
  elseif key_char_lower==("l") then targetvel_new[3]=targetvel[3]-0.15;
  elseif key_char_lower==(",") then targetvel_new[1]=targetvel[1]-0.05;
  elseif key_char_lower==("h") then targetvel_new[2]=targetvel[2]+0.02;
  elseif key_char_lower==(";") then targetvel_new[2]=targetvel[2]-0.02;

  end


  local vel_diff = (targetvel_new[1]-targetvel[1])^2+(targetvel_new[2]-targetvel[2])^2+(targetvel_new[3]-targetvel[3])^2
  if vel_diff>0 then
    targetvel[1],targetvel[2],targetvel[3] = targetvel_new[1],targetvel_new[2],targetvel_new[3]
    print(string.format("Target velocity: %.3f %.3f %.3f",unpack(targetvel)))
    mcm.set_walk_vel(targetvel)
  end

  ----------------------------------------------------------




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
