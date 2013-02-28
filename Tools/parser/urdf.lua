require 'include'
require 'LuaXml'
local util = require 'util'

local datapath = ''
--local filename = datapath..'atlas_simple_shapes.urdf' 
local filename = datapath..'atlas.urdf' 

local file = xml.load(filename)
local data = file:find('robot')
local robot = {}

robot.joints = {}
robot.links = {}
robot.name = data.name
if data ~= nil then
  for i = 1, #data do
    if data[i]:tag() == 'joint' then
      robot.joints[#robot.joints + 1] = data[i]
    elseif data[i]:tag() == 'link' then
      robot.links[#robot.links + 1] = data[i]
    end
  end
end

function str2num(str, n)
  local vals = string.gmatch(str, "%d+")
  local num = {}
  for i = 1, n do
    print(vals())
  end
end

print(robot.name)
print(#robot.joints)
print(#robot.links)
--for i = 1, #robot.joints do
--  print(robot.joints[i])
--end
--for i = 1, #robot.links do
--  print(robot.links[i])
--end

--for i = 1, #robot.joints do
for i = 1, 1 do
  local joint = robot.joints[i]
  print(joint.name, joint.type)
  for j = 1, #joint do
    joint[joint[j]:tag()] = joint[j]
  end
  print(joint.origin.xyz)
  str2num(joint.origin.xyz, 3)
--  print(joint.origin.rpy)
end
