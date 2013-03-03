--require 'common'
require 'include'

require 'LuaXml'
local util = require 'util'

--local datapath = '/home/yida/drcsim/ros/atlas_description/gazebo/atlas/meshes/' 
local datapath = 'powerplant/'
local filename = datapath..'model.sdf' 

local file = xml.load(filename)
local model = file:find('model')
local data = {}
if model ~= nil then
  for i = 1, #model do
    data[model[i]:tag()] = model[i]
  end
end

for k, v in pairs(data) do
  print(k)
end

