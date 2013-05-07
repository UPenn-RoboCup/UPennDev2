local libDynamixel = {}
local DynamixelPacket = require('DynamixelPacket');

-- Add a poor man's unix library
local unix = {}
unix.write = function(fd,msg)
	local str = string.format('%s fd:(%d) sz:(%d)',type(msg),fd or -1,#msg)
	local str2 = '\n'
	local str3 = '\n'
	for i=1,#msg do
		str2 = str2..string.format('%3d ', msg:byte(i))
		str3 = str3..string.format(' %.2X ', msg:byte(i))
	end
	print(str,str2,str3,'\n')
	return #msg
end
unix.time = function()
	return os.clock()
end
unix.read = function(fd)
	return nil
end
unix.usleep = function(n_usec)
	--print('sleeping!!',n_usec/1e6)
	os.execute('sleep '..n_usec/1e6)
end

libDynamixel.open = function( ttyname, ttybaud )
	-------------------------------
	-- Perform require upon an open
	unix = require('unix');
	stty = require('stty');
	local baud = ttybaud or 1000000;
	local fd = -1
	-------------------------------
	if not ttyname then
		local ttys = unix.readdir("/dev");
		for i=1,#ttys do
			if (string.find(ttys[i], "tty.usb") or
			string.find(ttys[i], "ttyUSB")) then
				ttyname = "/dev/"..ttys[i];
				break;
			end
		end
	end
	assert(ttyname, "Dynamixel tty not found");

	--print(string.format("Opening Dynamixel tty: %s\n", ttyname));

	-------------------------------
	-- Next line unneeded, since we are going to an object model...
	--if (fd >= 0) then unix.close(fd); end
	fd = unix.open(ttyname, unix.O_RDWR+unix.O_NOCTTY+unix.O_NONBLOCK);
	assert(fd >= 0, "Could not open port");
	-------------------------------

	-------------------------------
	-- Setup serial port parameters
	stty.raw(fd);
	stty.serial(fd);
	stty.speed(fd, baud);
	-------------------------------

	return fd, ttyname;
end

libDynamixel.close = function( fd )
	-- fd of 0,1,2 are stdin, stdout, sterr respectively
	if fd < 3 then
		return false
	end
	-- TODO: is there a return value here for errors/success?
	unix.close( fd );
	return true
end

libDynamixel.reset = function( fd, ttyname )
	--print("Reseting Dynamixel tty!");
	libDynamixel.close( fd );
	unix.usleep( 100000) ;
	libDynamixel.open(  fd, ttyname  );
	return true
end

libDynamixel.parse_status_packet = function( pkt )
	local t = {};
	t.id = pkt:byte(3);
	t.length = pkt:byte(4);
	t.error = pkt:byte(5);
	t.parameter = {pkt:byte(6,t.length+3)};
	t.checksum = pkt:byte(t.length+4);
	return t;
end

libDynamixel.get_status = function( fd, timeout )
	timeout = timeout or 0.010;
	local t0 = unix.time();
	local str = "";
	while unix.time()-t0 < timeout do
		local s = unix.read(fd);
		if (type(s) == "string") then
			str = str..s;
			pkt = DynamixelPacket.input(str);
			if (pkt) then
				local status = libDynamixel.parse_status_packet(pkt);
				--	    print(string.format("Status: id=%d error=%d",status.id,status.error));
				return status;
			end
		end
		unix.usleep(100);
	end
	return nil;
end

libDynamixel.send_ping = function( fd, id )
	local inst = DynamixelPacket.ping(id);
	return unix.write(fd, inst);
end

libDynamixel.ping_probe = function(fd, twait)
	twait = twait or 0.010;
	for id = 0,253 do
		io.write("Ping: Dynamixel ID ",id)
		libDynamixel.send_ping(fd, id);
		local status = libDynamixel.get_status(fd, twait);
		if status then
			io.write(status.id)
		end
	end
end

libDynamixel.set_delay = function(fd, id, value)
	local addr = 5;  -- Return Delay address
	local inst = DynamixelPacket.write_byte(id, addr, value);
	return unix.write(fd, inst);
end

libDynamixel.set_id = function(idOld, idNew)
	local addr = 3;  -- ID
	local inst = DynamixelPacket.write_byte(idOld, addr, idNew);
	return unix.write(fd, inst);
end

libDynamixel.set_led = function(fd, id, value)
	local addr = 25;  -- Led
	local inst = DynamixelPacket.write_byte(id, addr, value);
	return unix.write(fd, inst);
end

function libDynamixel.set_torque_enable(fd, id, value)
	local addr = 24;  -- Torque enable address
	local inst = DynamixelPacket.write_byte(id, addr, value);
	return unix.write(fd, inst);
end

libDynamixel.set_velocity = function(fd, id, value)
	local addr = 32; -- Moving speed address
	local inst = DynamixelPacket.write_word(id, addr, value);
	return unix.write(fd, inst);
end

function libDynamixel.set_hardness (fd, id, value)
	local addr = 34;  -- Torque limit address
	local inst = DynamixelPacket.write_word(id, addr, value);
	return unix.write(fd, inst)
end

libDynamixel.set_command = function(fd, id, value)
	local addr = 30;  -- Goal position address
	local inst = DynamixelPacket.write_word(id, addr, value);
	return unix.write(fd, inst)
end

libDynamixel.get_led = function(fd, id, value)
	local twait = 0.100;
	local addr = 25;  -- Led address
	local inst = DynamixelPacket.read_data(id, addr, 1);
	unix.read(fd); -- clear old status packets
	unix.write(fd, inst)
	local status = libDynamixel.get_status(fd, twait);
	if (status) then
		return status.parameter[1];
	end
end

libDynamixel.get_delay = function(fd, id, value)
	local twait = 0.100;
	local addr = 5;  -- Return delay address
	local inst = DynamixelPacket.read_data(id, addr, 1);
	unix.read(fd); -- clear old status packets
	unix.write(fd, inst)
	local status = libDynamixel.get_status(fd, twait);
	if (status) then
		return status.parameter[1];
	end
end

libDynamixel.get_torque_enable = function(fd, id, value)
	local twait = 0.100;
	local addr = 24;  -- Torque enable address
	local inst = DynamixelPacket.read_data(id, addr, 1);
	unix.read(fd); -- clear old status packets
	unix.write(fd, inst)
	local status = libDynamixel.get_status(fd, twait);
	if (status) then
		return status.parameter[1];
	end
end

libDynamixel.get_position = function(fd, id, value)
	local twait = 0.100;
	local addr = 36;  -- Present position address
	local inst = DynamixelPacket.read_data(id, addr, 2);
	unix.read(fd); -- clear old status packets
	unix.write(fd, inst)
	local status = libDynamixel.get_status(fd, twait);
	if (status) then
		return DynamixelPacket.byte_to_word(unpack(status.parameter,1,2));
	end
end

libDynamixel.get_command = function(fd, id, value)
	local twait = 0.100;
	local addr = 30;  -- Goal position address
	local inst = DynamixelPacket.read_data(id, addr, 2);
	unix.read(fd); -- clear old status packets
	unix.write(fd, inst)
	local status = libDynamixel.get_status(fd, twait);
	if (status) then
		return DynamixelPacket.byte_to_word(unpack(status.parameter,1,2));
	end
end

libDynamixel.get_velocity = function(fd, id, value)
	local twait = 0.100;
	local addr = 32; -- Moving speed address
	local inst = DynamixelPacket.read_data(id, addr, 2);
	unix.read(fd); -- clear old status packets
	unix.write(fd, inst)
	local status = libDynamixel.get_status(fd, twait);
	if (status) then
		return DynamixelPacket.byte_to_word(unpack(status.parameter,1,2));
	end
end

libDynamixel.get_hardness = function(fd, id, value)
	local twait = 0.100;
	local addr = 34;  -- Torque limit address
	local inst = DynamixelPacket.read_data(id, addr, 2);
	unix.read(fd); -- clear old status packets
	unix.write(fd, inst)
	local status = libDynamixel.get_status(fd, twait);
	if (status) then
		return DynamixelPacket.byte_to_word(unpack(status.parameter,1,2));
	end
end

libDynamixel.get_battery = function(fd, id, value)
	local twait = 0.100;
	local addr = 42;  -- Present voltage address
	local inst = DynamixelPacket.read_data(id, addr, 1);
	unix.read(fd); -- clear old status packets
	unix.write(fd, inst)
	local status = libDynamixel.get_status(fd, twait);
	if (status) then
		return status.parameter[1];
	end
end

libDynamixel.get_temperature = function(fd, id, value)
	local twait = 0.100;
	local addr = 43;  -- Present Temperature
	local inst = DynamixelPacket.read_data(id, addr, 1);
	unix.read(fd); -- clear old status packets
	unix.write(fd, inst)
	local status = libDynamixel.get_status(fd, twait);
	if (status) then
		return status.parameter[1];
	end
end

libDynamixel.read_data = function(fd, addr, len, twait)
	twait = twait or 0.100;
	len  = len or 2;
	local inst = DynamixelPacket.read_data(id, addr, len);
	unix.read(fd); -- clear old status packets
	unix.write(fd, inst)
	local status = libDynamixel.get_status(fd, twait);
	if (status) then
		return status.parameter;
	end
end

-- TODO: not supported yet...?
libDynamixel.bulk_read_data = function(fd, id_cm730, ids, addr, len, twait)
	twait = twait or 0.100;
	len  = len or 2;
	local inst = DynamixelPacket.bulk_read_data(
	id_cm730,string.char(unpack(ids)), addr, len, #ids);
	unix.read(fd); -- clear old status packets
	unix.write(fd, inst)
	local status = libDynamixel.get_status(fd, twait);
	if (status) then
		return status.parameter;
	end
end

libDynamixel.sync_write_byte = function(fd, ids, addr, data)
	local nid = #ids;
	local len = 1;

	local t = {};
	local n = 1;
	for i = 1,nid do
		t[n] = ids[i];
		t[n+1] =  data[i];
		n = n + len + 1;
	end
	local inst = DynamixelPacket.sync_write(addr, len,
	string.char(unpack(t)));
	unix.write(fd, inst);
end

libDynamixel.sync_write_word = function( fd, ids, addr, data )
	local nid = #ids;
	local len = 2;

	local t = {};
	local n = 1;
	for i = 1,nid do
		t[n] = ids[i];
		t[n+1],t[n+2] = DynamixelPacket.word_to_byte(data[i]);
		n = n + len + 1;
	end
	local inst = DynamixelPacket.sync_write(addr, len,
	string.char(unpack(t)));
	unix.write(fd, inst);
end

-- Convert the message type
--[[
libDynamixel.normal2pro() = function end
libDynamixel.pro2normal() = function end
libDynamixel.normal2hybrid() = function end
libDynamixel.hybrid2normal() = function end
--]]

return libDynamixel