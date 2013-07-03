dofile('../include.lua')
local unix = require'unix'
local mp = require 'messagepack'
local udp = require 'udp'
local udp_ok = udp.init('255.255.255.255', 54321);
assert(udp_ok,"Bad udp setup!")

local offset_l = {}
local offset_r = {}
local t0 = unix.time()
local cnt = 0;
while true do
	if udp.size()>0 then
		local data = udp.receive()
		local msg = mp.unpack(data);
		offset_l = msg[1];
		offset_r = msg[2];
	end
	local t = unix.time()
	local t_diff = t-t0
	cnt=cnt+1
	if t_diff>1 then
		local fps = cnt/t_diff;
		print( string.format("%.2f FPS",fps) )
		t0 = t
		cnt=0
		if #offset_l==3 then
			print( string.format('Left : %.3f %.3f %.3f',unpack(offset_l)) );
		end
		if #offset_r==3 then
			print( string.format('Right: %.3f %.3f %.3f',unpack(offset_r)) );
		end
	end
end
