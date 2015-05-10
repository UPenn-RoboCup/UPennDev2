#!/usr/local/bin/luajit
local f = io.open('jacobian_com_waist.txt','r')
local x = f:read('*all')
f:close()

local jac = {}
local j_ids = {}
local function simp_trig(trig, j)
  local id = tonumber(j)
  j_ids[id] = true
  return trig:sub(1, 1)..id
end
-- Remove the zeros...
local matrix = x
:gsub('(%l%l%l)%(q(%d)%)', simp_trig)
:gsub('0]', ']') -- The end zeros mean nothing, then
:gsub(',',',\n')
:gsub('%]%)','\n}')
:gsub('Matrix%(%[','\n{')
:gsub('%[', '\n{')
:gsub('%]', '\n}')

table.insert(jac, 'local function jac(q)')
for id in pairs(j_ids) do
  table.insert(jac, string.format('local c%d, s%d = cos(q[%d]), sin(q[%d])', id, id, id, id))
end
table.insert(jac, 'return '..matrix)
table.insert(jac, 'end')
table.insert(jac, 'cos, sin = math.cos, math.sin')
table.insert(jac, 'upperArmLength, elbowOffsetX, lowerArmLength, shoulderOffsetY = 0.246,0.03,0.25, 0.234')
table.insert(jac, 'l_8x, l_8y, l_8z = 0,0,0')


table.insert(jac, 'local J = jac({0, 0,0,0, 0, 0,0,0})')
table.insert(jac, 'print(#J)')
table.insert(jac, 'for i, r in ipairs(J) do print(#r..":", unpack(r)) end')

print(table.concat(jac,'\n'))
