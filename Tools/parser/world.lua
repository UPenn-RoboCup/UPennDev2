require 'include'
require 'LuaXml'
local util = require 'util'

require 'vrml'

header = '#VRML_SIM V6.0 utf8'
local world = createVRML(header)

world[1] = createNode(_, 'WorldInfo')
world[2] = createNode(_, 'Viewpoint')
world[3] = createNode(_, 'Background')
world[4] = createNode(_, 'PointLight')

info = {'Description','Author: first name last name <e-mail>','Date: DD MMM YYYY'}
world[1][1] = createMultiField('info', info)

orientation = {1,0,0,-0.8}
world[2][1] = createField('orientation', orientation)
position = {0.25,0.708035,0.894691}
world[2][2] = createField('position',position)

skyColor = {0.4, 0.7, 1}
world[3][1] = createMultiField('skyColor', skyColor)

ambientIntensity = {0.54}
world[4][1] = createField('ambientIntensity', ambientIntensity)
intensity = {0.5}
world[4][2] = createField('intensity', intensity)
location = {0, 1, 0}
world[4][3] = createField('location', location)


saveVRML(world, 'aa.wbt')
