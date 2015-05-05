local f = io.open('jacobian.txt','r')
local x = f:read('*all')
f:close()

local jac = {}
local j_ids = {}
local function simp_trig(trig, j)
  local id = tonumber(j)
  j_ids[id] = true
  return trig:sub(1, 1)..id
end
local matrix = x:gsub('(%l%l%l)%(q(%d)%)', simp_trig):gsub('%]%)','}'):gsub('Matrix%(%[','{'):gsub('%[', '{'):gsub('%]', '}')

table.insert(jac, 'local function jac(q)')
for id in pairs(j_ids) do
  table.insert(jac, string.format('local c%d, s%d = cos(q[%d]), sin(q[%d])', id, id, id, id))
end
table.insert(jac, 'return '..matrix)
table.insert(jac, 'end')

print(table.concat(jac,'\n'))
