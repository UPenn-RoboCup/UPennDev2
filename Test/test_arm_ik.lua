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



--local yawmin,yawmax,yawdiv = -85*DEG_TO_RAD,85*DEG_TO_RAD,5*DEG_TO_RAD
--local div = 0.01

local yawmin,yawmax,yawdiv = -90*DEG_TO_RAD,90*DEG_TO_RAD,10*DEG_TO_RAD
local div = 0.02

--8 sec with 0.02/10 deg div
--8*8*2 sec with 0.01/5 deg div


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

print("x:",xmin,xmax,div)
print("y:",ymin,ymax,div)
print("z:",zmin,zmax,div)
print("yaw:",yawmin/DEG_TO_RAD,yawmax/DEG_TO_RAD,yawdiv/DEG_TO_RAD)


local count, poscount=0,0


local outstr=""
local outcount=0

local x_max_m,y_max_m,z_max_m = 0,0,0


for x=xmin,xmax,div do	
	if x>x_max_m then x_max_m=x end
	for y=ymin,ymax,div do
		if y>y_max_m then y_max_m=y end
		for z=zmin,zmax,div do
			if z>z_max_m then z_max_m=z end
			poscount=poscount+1

			local max_margin,max_margin_yaw = 0,nil

			local start_count=0
			local end_count=0
			local last_valid=false

			local str=""

			for yaw=yawmin,yawmax,yawdiv do
				count=count+1
				local trArm = {x,y,z,0,0,45*math.pi/180}
				local qArm = Body.get_inverse_rarm(vector.zeros(7),
					trArm, yaw,0,{0,0})
				local margin = arm_planner.calculate_margin(qArm,0,trArm)

				if margin>0 then
					if not last_valid then start_count=start_count+1 end
					last_valid = true
				else
					if last_valid then end_count=end_count+1 end
					last_valid = false
				end

				if margin>max_margin then
				  max_margin = margin
				  max_margin_yaw = yaw
				end

--			if qArmNext then print("ok") 
--		  else print("bad")		end
				margin=math.max(0,margin)
--				str=str..string.format("%d ",margin/DEG_TO_RAD)
			end
			--print(string.format("%.2f %.2f %.2f  ",x,y,z)..str)
			--if start_count>1 then print("BIMODAL",x,y,z) end
			if not max_margin_yaw then max_margin_yaw=99*DEG_TO_RAD end
  	  outstr=outstr..string.format("%d,",max_margin_yaw/DEG_TO_RAD)
			outcount=outcount+1
			if outcount%10==0 then outstr=outstr.."\n" end
		end
	end
	local percent = (x-xmin)/(xmax-xmin)*100
	print(string.format("%.1f percent done",percent))
end

print("x_max,y_max:",x_max_m,y_max_m)




outfile=assert(io.open("../Config/THOROP/iklookup.lua","w"));

local outstr1="module(..., package.seeall)\n\n"..
						 "iklookup={}\n"..
						 string.format("iklookup.x={%.2f,%.2f,%.2f}\n",xmin,x_max_m,div)..
						 string.format("iklookup.y={%.2f,%.2f,%.2f}\n",ymin,y_max_m,div)..
						 string.format("iklookup.z={%.2f,%.2f,%.2f}\n",zmin,zmax,div)..
						 "iklookup.dat={\n"..outstr

outfile:write(outstr1.."\n}");
outfile:flush();
outfile:close();

local t1 =unix.time()
print(count,' total positions tested')
print(poscount,' positions tested')
print((t1-t0), 'sec passed')
