module(..., package.seeall)
local vector = require'vector'

cal={}
cal[HOSTNAME] = {}
cal["asus"] = {}
cal["alvin"]={}



-- Updated date: Sat Jun 21 20:14:27 2014
cal["asus"].legBias=vector.new({
   2.024974,0.000000,0.000000,0.000000,0.000000,0.000000,
   0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,
   })*math.pi/180


-- Updated date: Sun Jun 22 19:06:55 2014
cal["alvin"].legBias=vector.new({
   0,0,0,0,0,0,
   0,0,0,0,0,0,
   })*math.pi/180




-- Updated date: Sun Jun 22 19:41:19 2014
--Front is kinda stable
cal["alvin"].legBias=vector.new({
   1.417500,0.742500,-1.417500,-1.552500,-0.540000,0.472500,
   0.472500,-2.025000,0.000000,-0.810000,0.000000,1.417500,
   })*math.pi/180
