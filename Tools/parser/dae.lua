require 'common'
require 'include'

require 'LuaXml'
local util = require 'util'

local datapath = '/home/yida/drcsim/ros/atlas_description/gazebo/atlas/meshes/' 
local filename = datapath..'head.dae' 

local daefile = xml.load(filename)

local collada = daefile:find('COLLADA')
if collada ~= nil then
--  print(collada)
    print(collada[1]:tag(), #collada[1], collada[1])
    print(collada[1][8])
    util.ptable(collada[1][8]) 
    type(collada[1][8].meter)
end

