require 'include'
require 'LuaXml'
require 'vrml'
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
print('joints # '..#robot.joints)
print('links # '..#robot.links)
--for i = 1, #robot.joints do
--  print(robot.joints[i])
--end
for i = 1, #robot.links do
--  print(robot.links[i].name)
  robot.links[robot.links[i].name] = robot.links[i]
end

local bodyNode = {}
local partsNum = 0
for i = 1, #robot.joints do
--for i = 1, 1 do
  local joint = robot.joints[i]
  for j = 1, #joint do
    joint[joint[j]:tag()] = joint[j]
  end
  -- put joint in bodyNode
  local jointTag = joint.child.link..'+'..joint.parent.link
  if bodyNode[jointTag] == nil then
    bodyNode[jointTag] = {['next']={}, ['prev']={}}
    bodyNode[jointTag].next[#bodyNode[jointTag].next + 1] = joint.child.link
    bodyNode[jointTag].prev[#bodyNode[jointTag].prev + 1] = joint.parent.link
  else
    error('duplicated joint')
  end
  -- put link in bodyNode
  local linkTag = joint.child.link
  if bodyNode[linkTag] == nil then
    bodyNode[linkTag] = {['next']={}, ['prev']={}}
  end
  bodyNode[linkTag].prev[#bodyNode[linkTag].prev + 1] = jointTag
--  print(joint.child.link)
  local linkTag = joint.parent.link
  if bodyNode[linkTag] == nil then
    bodyNode[linkTag] = {['next']={}, ['prev']={}}
  end
  bodyNode[linkTag].next[#bodyNode[linkTag].next + 1] = jointTag
--  print(joint.parent.link)

end

local root = {}
for k, v in pairs(bodyNode) do
  local startNode = k
  while #bodyNode[startNode].prev ~= 0 do
--    print(#bodyNode[startNode].prev)
    startNode = bodyNode[startNode].prev[1]
  end
  if #root == 0 or root[#root] ~= startNode then
    root[#root+1] = startNode
  end
--  print('end with '..startNode);
end

assert(#root == 1)

function bodyTraverse(rootNode, body, indent)
  for i = 1, #body[rootNode].next do
    print(indentSpace(indent)..body[rootNode].next[i])
    bodyTraverse(body[rootNode].next[i], body, indent+1)
  end
end

bodyTraverse(root[1], bodyNode, 0)
