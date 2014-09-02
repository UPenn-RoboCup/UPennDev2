local f = io.open('fk.txt','r')
local x = f:read('*all')
f:close()

local fks = {
  'local K = {}',
  'local sin, cos = math.sin, math.cos',
  'local function fk(q)',
}
table.insert(fks,
[[
local shoulderOffsetX = 0;    
local shoulderOffsetY = 0.234;
local shoulderOffsetZ = 0.165;
local upperArmLength = .246;
local elbowOffsetX =   .030; 
--local lowerArmLength = .186; //Default 7DOF arm
local lowerArmLength = .250; -- LONGARM model
]]
)

local j_ids = {}
local function simp_trig(trig, j)
  local id = tonumber(j)
  j_ids[id] = true
  return trig:sub(1, 1)..id
end
local matrix = x:gsub('(%l%l%l)%(q(%d)%)', simp_trig):gsub('%]%)','}'):gsub('Matrix%(%[','{'):gsub('%[', '{'):gsub('%]', '}')
for id in pairs(j_ids) do
  table.insert(fks, string.format('local c%d, s%d = cos(q[%d]), sin(q[%d])', id, id, id, id))
end

table.insert(fks, 'local T = '..matrix)
table.insert(fks, 'return torch.Tensor(T)')
--table.insert(fks, 'return T')
table.insert(fks, 'end')
table.insert(fks, 'K.fk = fk')

table.insert(fks, 'return K')

print(table.concat(fks,'\n'))

--[[
Matrix([
[(((-sin(q1)*cos(q3) + sin(q2)*sin(q3)*cos(q1))*cos(q4) + sin(q4)*cos(q1)*cos(q2))*sin(q5) - (-sin(q1)*sin(q3) - sin(q2)*cos(q1)*cos(q3))*cos(q5))*sin(q6) - (-(-sin(q1)*cos(q3) + sin(q2)*sin(q3)*cos(q1))*sin(q4) + cos(q1)*cos(q2)*cos(q4))*cos(q6), -((((-sin(q1)*cos(q3) + sin(q2)*sin(q3)*cos(q1))*cos(q4) + sin(q4)*cos(q1)*cos(q2))*sin(q5) - (-sin(q1)*sin(q3) - sin(q2)*cos(q1)*cos(q3))*cos(q5))*cos(q6) + (-(-sin(q1)*cos(q3) + sin(q2)*sin(q3)*cos(q1))*sin(q4) + cos(q1)*cos(q2)*cos(q4))*sin(q6))*sin(q7) + (-((-sin(q1)*cos(q3) + sin(q2)*sin(q3)*cos(q1))*cos(q4) + sin(q4)*cos(q1)*cos(q2))*cos(q5) - (-sin(q1)*sin(q3) - sin(q2)*cos(q1)*cos(q3))*sin(q5))*cos(q7), ((((-sin(q1)*cos(q3) + sin(q2)*sin(q3)*cos(q1))*cos(q4) + sin(q4)*cos(q1)*cos(q2))*sin(q5) - (-sin(q1)*sin(q3) - sin(q2)*cos(q1)*cos(q3))*cos(q5))*cos(q6) + (-(-sin(q1)*cos(q3) + sin(q2)*sin(q3)*cos(q1))*sin(q4) + cos(q1)*cos(q2)*cos(q4))*sin(q6))*cos(q7) + (-((-sin(q1)*cos(q3) + sin(q2)*sin(q3)*cos(q1))*cos(q4) + sin(q4)*cos(q1)*cos(q2))*cos(q5) - (-sin(q1)*sin(q3) - sin(q2)*cos(q1)*cos(q3))*sin(q5))*sin(q7), -0.03*((-sin(q1)*cos(q3) + sin(q2)*sin(q3)*cos(q1))*cos(q4) + sin(q4)*cos(q1)*cos(q2))*sin(q5) + 0.03*(-sin(q1)*sin(q3) - sin(q2)*cos(q1)*cos(q3))*cos(q5) + 0.25*(-sin(q1)*cos(q3) + sin(q2)*sin(q3)*cos(q1))*sin(q4) + 0.03*(-sin(q1)*cos(q3) + sin(q2)*sin(q3)*cos(q1))*cos(q4) + 0.03*sin(q4)*cos(q1)*cos(q2) - 0.25*cos(q1)*cos(q2)*cos(q4) + 0.246*cos(q1)*cos(q2)],
[(((sin(q1)*sin(q2)*sin(q3) + cos(q1)*cos(q3))*cos(q4) + sin(q1)*sin(q4)*cos(q2))*sin(q5) - (-sin(q1)*sin(q2)*cos(q3) + sin(q3)*cos(q1))*cos(q5))*sin(q6) - (-(sin(q1)*sin(q2)*sin(q3) + cos(q1)*cos(q3))*sin(q4) + sin(q1)*cos(q2)*cos(q4))*cos(q6), -((((sin(q1)*sin(q2)*sin(q3) + cos(q1)*cos(q3))*cos(q4) + sin(q1)*sin(q4)*cos(q2))*sin(q5) - (-sin(q1)*sin(q2)*cos(q3) + sin(q3)*cos(q1))*cos(q5))*cos(q6) + (-(sin(q1)*sin(q2)*sin(q3) + cos(q1)*cos(q3))*sin(q4) + sin(q1)*cos(q2)*cos(q4))*sin(q6))*sin(q7) + (-((sin(q1)*sin(q2)*sin(q3) + cos(q1)*cos(q3))*cos(q4) + sin(q1)*sin(q4)*cos(q2))*cos(q5) - (-sin(q1)*sin(q2)*cos(q3) + sin(q3)*cos(q1))*sin(q5))*cos(q7), ((((sin(q1)*sin(q2)*sin(q3) + cos(q1)*cos(q3))*cos(q4) + sin(q1)*sin(q4)*cos(q2))*sin(q5) - (-sin(q1)*sin(q2)*cos(q3) + sin(q3)*cos(q1))*cos(q5))*cos(q6) + (-(sin(q1)*sin(q2)*sin(q3) + cos(q1)*cos(q3))*sin(q4) + sin(q1)*cos(q2)*cos(q4))*sin(q6))*cos(q7) + (-((sin(q1)*sin(q2)*sin(q3) + cos(q1)*cos(q3))*cos(q4) + sin(q1)*sin(q4)*cos(q2))*cos(q5) - (-sin(q1)*sin(q2)*cos(q3) + sin(q3)*cos(q1))*sin(q5))*sin(q7), -0.03*((sin(q1)*sin(q2)*sin(q3) + cos(q1)*cos(q3))*cos(q4) + sin(q1)*sin(q4)*cos(q2))*sin(q5) + 0.25*(sin(q1)*sin(q2)*sin(q3) + cos(q1)*cos(q3))*sin(q4) + 0.03*(sin(q1)*sin(q2)*sin(q3) + cos(q1)*cos(q3))*cos(q4) + 0.03*(-sin(q1)*sin(q2)*cos(q3) + sin(q3)*cos(q1))*cos(q5) + 0.03*sin(q1)*sin(q4)*cos(q2) - 0.25*sin(q1)*cos(q2)*cos(q4) + 0.246*sin(q1)*cos(q2)],
[((-sin(q2)*sin(q4) + sin(q3)*cos(q2)*cos(q4))*sin(q5) + cos(q2)*cos(q3)*cos(q5))*sin(q6) - (-sin(q2)*cos(q4) - sin(q3)*sin(q4)*cos(q2))*cos(q6), -(((-sin(q2)*sin(q4) + sin(q3)*cos(q2)*cos(q4))*sin(q5) + cos(q2)*cos(q3)*cos(q5))*cos(q6) + (-sin(q2)*cos(q4) - sin(q3)*sin(q4)*cos(q2))*sin(q6))*sin(q7) + (-(-sin(q2)*sin(q4) + sin(q3)*cos(q2)*cos(q4))*cos(q5) + sin(q5)*cos(q2)*cos(q3))*cos(q7), (((-sin(q2)*sin(q4) + sin(q3)*cos(q2)*cos(q4))*sin(q5) + cos(q2)*cos(q3)*cos(q5))*cos(q6) + (-sin(q2)*cos(q4) - sin(q3)*sin(q4)*cos(q2))*sin(q6))*cos(q7) + (-(-sin(q2)*sin(q4) + sin(q3)*cos(q2)*cos(q4))*cos(q5) + sin(q5)*cos(q2)*cos(q3))*sin(q7), -0.03*(-sin(q2)*sin(q4) + sin(q3)*cos(q2)*cos(q4))*sin(q5) - 0.03*sin(q2)*sin(q4) + 0.25*sin(q2)*cos(q4) - 0.246*sin(q2) + 0.25*sin(q3)*sin(q4)*cos(q2) + 0.03*sin(q3)*cos(q2)*cos(q4) - 0.03*cos(q2)*cos(q3)*cos(q5)],
[0, 0, 0, 1]
])
--]]

--[[
x:gsub('cos%(q(%d)%)', 'c%1'):gsub('sin%(q(%d)%)', 's%1')

cs = {}
y = string.gsub(x, 'cos%(q(%d)%)', function(j) local n = tonumber(j); cs[n]=true; return 'c'..(n-1) end)

x:gsub('cos%(q(%d)%)', 'c%1'):gsub('sin%(q(%d)%)', 's%1')

return string.gsub(x, '(%l%l%l)%(q(%d)%)', '%1 %2')
--]]