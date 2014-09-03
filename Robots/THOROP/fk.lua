local f = io.open('fk.txt','r')
local x = f:read('*all')
f:close()

local fks = {
  'local K = {}',
  'local sin, cos = math.sin, math.cos',
  [[
  local shoulderOffsetX = 0;    
  local shoulderOffsetY = 0.234;
  local shoulderOffsetZ = 0.165;
  local upperArmLength = .246;
  local elbowOffsetX =   .030; 
  --local lowerArmLength = .186; //Default 7DOF arm
  local lowerArmLength = .250; -- LONGARM model
  ]],
}
table.insert(fks, 'local torch = require"torch"')
local j_ids = {}
local function simp_trig(trig, j)
  local id = tonumber(j)
  j_ids[id] = true
  return trig:sub(1, 1)..id
end
local matrix = x:gsub('(%l%l%l)%(q(%d)%)', simp_trig):gsub('%]%)','}'):gsub('Matrix%(%[','{'):gsub('%[', '{'):gsub('%]', '}')

table.insert(fks, 'local function fk(q)')
for id in pairs(j_ids) do
  table.insert(fks, string.format('local c%d, s%d = cos(q[%d]), sin(q[%d])', id, id, id, id))
end
table.insert(fks, 'return '..matrix)
table.insert(fks, 'end')

-- Now add the faster torch
table.insert(fks, 'local function fk_t(tr_d, q)')
for id in pairs(j_ids) do
  table.insert(fks, string.format('local c%d, s%d = cos(q[%d]), sin(q[%d])', id, id, id, id))
end
local s = matrix:find('{')
local e = #matrix + 1 - matrix:reverse():find('}')
local i = 0
for row in matrix:sub(s+1, e-1):gmatch('%{%C*%}') do
  for entry in row:gsub('[%s%{%}]+', ''):gmatch('[^,]+') do
    table.insert(fks, string.format('tr_d[%d] = %s', i, entry))
    i = i + 1
  end
end
table.insert(fks, 'end')
--local r1, r2, r3, r4 = m(), m(), m(), m()

-- Export the object
table.insert(fks, 'K.fk = fk')
table.insert(fks, 'K.fk_t = fk_t')
table.insert(fks, 'return K')

print(table.concat(fks,'\n'))
