--require 'common'
require 'include'

require 'LuaXml'
local util = require 'util'

--local datapath = '/home/yida/drcsim/ros/atlas_description/gazebo/atlas/meshes/' 
local datapath = 'powerplant/meshes/'
local filename = datapath..'powerplant.dae' 

local daefile = xml.load(filename)
local collada = daefile:find('COLLADA')
local dae = {}
if collada ~= nil then
  for i = 1, #collada do
    dae[collada[i]:tag()] = collada[i]
  end
end

print('library_nodes:'..#dae.library_nodes)
print('library_images:'..#dae.library_images)
print('library_effects:'..#dae.library_effects)
print('library_materials:'..#dae.library_materials)
print('library_geometries:'..#dae.library_geometries)
print('library_visual_scenes:'..#dae.library_visual_scenes)

--for k, v in pairs(dae) do
--  print(k)
--end
--scene
--library_materials
--library_images
--asset
--library_effects
--library_nodes
--library_geometries
--library_visual_scenes
