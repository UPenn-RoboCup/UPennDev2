#!/usr/local/bin/luajit
require'rrts'

local gamma = 0.75
local nIter = 1e4

print("Solving with gamma=", gamma)
local plan = {
  qArm0 = {1.91986, 0, 0, -2.7, 0, 0.785398, 0},
  --trGoal = {0.46, 0.27, 0.3, 0, 0, -math.pi/3},
  tr = {
    {1, 0, 0, 0.46},
    {0, 1, 0, 0.27},
    {0, 0, 1, 0.3},
    {0, 0, 0, 1},
    },
  qWaistArmGuess = {-0.349, 1.047, -1.57, -2.094, 0, -0.785, 0},
}
local planner = {
  qMid = {0.785398, 0.759218, 0.392699, -1.39626, 0, 0, 0},
  qRange = {4.71239, 1.51844, 3.92699, 2.79253, 6.28319, 3.03687, 6.28319}
}

for i=1,5 do
  path = rrts.plan(plan, planner, gamma, nIter)
  --[[
  path = rrts.plan(
    {1.91986, 0, 0, -2.7, 0, 0.785398, 0}, -- start
    {0.46, 0.27, 0.3, 0, 0, -math.pi/3}, -- fk target
    {-0.349, 1.047, -1.57, -2.094, 0, -0.785, 0}, -- goal
    {0.785398, 0.759218, 0.392699, -1.39626, 0, 0, 0}, --mid
    {4.71239, 1.51844, 3.92699, 2.79253, 6.28319, 3.03687, 6.28319}, --range
    gamma,
    1e4
  )
--]]
  if path then break end
  
end

if type(path)=='table' then
  print("path", path, #path)
  for i, v in ipairs(path)
  do
    if type(v)=='table' then
      io.write(i, ': {', table.concat(v, ', '), '}', '\n')
    else
      print(i, type(v), v)
    end
  end
end