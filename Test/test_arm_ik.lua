#!/usr/bin/env luajit
-- (c) 2014 Team THORwIn
local ok = pcall(dofile,'../fiddle.lua')
if not ok then dofile'fiddle.lua' end

local targetvel = {0,0,0}
local targetvel_new = {0,0,0}
local WAS_REQUIRED

local t_last = Body.get_time()
local tDelay = 0.005*1E6

local command1, command2 = '',''  

local last_support = 0
local last_input = Body.get_time()
local input_delay = 1.0


local function show_status()

end

local function update(key_code)

end

show_status()
if ... and type(...)=='string' then
	WAS_REQUIRED = true
	return {entry=nil, update=update, exit=nil}
end


local getch = require'getch'
local vector = require'vector'
local running = true
local key_code

local libArmPlan=require'libArmPlan'

local arm_planner = libArmPlan.new_planner()
local unix = require'unix'
--right arm

local DEG_TO_RAD = math.pi/180

local xmin = 0.10
local xmax = 0.62
local ymin = -0.46
local ymax = 0.10
local zmin = -0.26
local zmax = 0.20
local div = 0.01


local yawmin = -85*DEG_TO_RAD
local yawmax = 85*DEG_TO_RAD


local t0 =unix.time()



local function gen_index(x,y,z)

	x = (math.max(xmin,math.min(xmax,x)) - xmin)/div
	y = (math.max(ymin,math.min(ymax,y)) - ymin)/div
	z = (math.max(zmin,math.min(zmax,z)) - zmin)/div
	--values: now 0 to (xmax-xmin)/div
	--or 0 to 
	local xi = math.floor(x+0.5)
  local yi = math.floor(x+0.5)
  local zi = math.floor(x+0.5)

  
end




local count=0
for x=xmin,xmax,0.01 do
	for y=ymin,ymax,0.01 do
		for z=zmin,zmax,0.01 do
			local max_margin,max_margin_yaw = 0,nil
			for yaw=yawmin,yawmax,5*DEG_TO_RAD do
				count=count+1
				local trArm = {x,y,z,0,0,45*math.pi/180}
				local qArm = Body.get_inverse_rarm(vector.zeros(7),
					trArm, yaw,0,{0,0})
				local margin = arm_planner.calculate_margin(qArm,0,trArm)

				if margin>max_margin then
				  max_margin = margin
				  max_margin_yaw = yaw
				end
--			if qArmNext then print("ok") 
--		  else print("bad")		end
		end
	end
end

local t1 =unix.time()
print(count,' positions tested')
print((t1-t0), 'sec passed')
