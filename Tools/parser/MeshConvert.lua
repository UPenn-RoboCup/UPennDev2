--require 'common'
--require 'include'

--require 'LuaXml'
--local util = require 'util'
local vcglib = require 'vcglib'

--local datapath = '/home/yida/drcsim/ros/atlas_description/gazebo/atlas/meshes/' 
--local datapath = 'powerplant/meshes/' 
local datapath = '' 
local filename = datapath..'head.stl' 
--local filename = datapath..'model.sdf' 
--local filename = datapath..'powerplant.stl' 

local drcsimPath = '../../../drcsim'

vcglib.stl2vrml(filename)
--vcglib.dae2vrml(filename)
--vcglib.sdf2vrml(filename)

-- gazebo_model
-- ros
-- world

--local daefile = xml.load(filename)
--
--local collada = daefile:find('COLLADA')
--if collada ~= nil then
----  print(collada)
--    print(collada[1]:tag(), #collada[1], collada[1])
--    print(collada[1][8])
--    util.ptable(collada[1][8]) 
--    type(collada[1][8].meter)
--end

