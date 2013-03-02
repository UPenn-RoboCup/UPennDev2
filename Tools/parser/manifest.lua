require 'include'
require 'LuaXml'
local util = require 'util'

local datapath = 'powerplant/'
local filename = datapath..'manifest.xml' 

local file = xml.load(filename)
local data = file:find('model')
local model = {}
if data ~= nil then
  for i = 1, #data do
    model[data[i]:tag()] = data[i]
  end
end

function showInfo(tbl)
  if tbl then
    for k, v in ipairs(tbl) do
      if type(v) == 'table' then
        showInfo(v)
      else
        print(v)
      end
    end
  end
end

showInfo(model.name)
showInfo(model.version)
showInfo(model.license)
showInfo(model.description)
showInfo(model.author)

if model.sdf then
  for i = 1, #model.sdf do
    print('sdf - '..model.sdf[i])
  end
end

--if model.depend then
--  for i = 1, #model.depend do
--    print(model.depend[i])
--  end
--end
