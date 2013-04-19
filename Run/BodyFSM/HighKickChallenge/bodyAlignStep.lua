module(..., package.seeall);

local Body = require('Body')
local vector = require('vector')
local Motion = require('Motion');
local kick = require('kick');
local HeadFSM = require('HeadFSM')
local Config = require('Config')
local wcm = require('wcm')
local walk = require('walk');
local align = require('align')

function entry()
  print(_NAME.." entry");
  t0 = Body.get_time();
  Motion.event("align");
  walk.stop();
end

function update()
  t = Body.get_time();
  if (t - t0 > 4.0) and not align.active then
    return "done";
  end
end

function exit()
end
