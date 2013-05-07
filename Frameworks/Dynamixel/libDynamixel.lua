local libDynamixel = {}
local DynamixelPacket = require('DynamixelPacket');

local function convert_hybrid(msg)
	-- Header
	local msg2 = string.char(255,255,253)
	-- Stuffing
	--msg2 = msg2..string.char(253)
	msg2 = msg2..string.char(0)
	-- Set servo id
	local servo_id = msg:byte(3)
	msg2 = msg2..string.char(servo_id)
	-- Set length
	local msg_len = #msg-2
	local len_hi = math.floor(msg_len/256)
	local len_lo = msg_len%256
	msg2 = msg2..string.char(len_lo,len_hi)
	-- Copy message
	-- TODO: THIS IS HACKY
	for pp=5,#msg-1 do
		local old_b = msg:byte(pp)
		msg2 = msg2..string.char(old_b)
	end
	msg2 = msg2..string.char(0)
	-- Add crc
	local lo,hi = DynamixelPacket.crc16(msg2)
	msg2 = msg2..string.char(lo,hi)
	return msg2
end

-- Add a poor man's unix library
local unix = {}
unix.write = function(fd,msg)
	local str = string.format('%s fd:(%d) sz:(%d)',type(msg),fd,#msg)
	local msg2 = convert_hybrid(msg)
	local str2 = 'Dec:\t'
	local str3 = 'Hex:\t'
	local str4 = 'HybD:\t'
	local str5 = 'HybH:\t'
	for i=1,#msg do
		str2 = str2..string.format('%3d ', msg:byte(i))
		str3 = str3..string.format(' %.2X ', msg:byte(i))
	end
	for i=1,#msg2 do
		str4 = str4..string.format('%3d ', msg2:byte(i))
		str5 = str5..string.format(' %.2X ', msg2:byte(i))
	end
	io.write(str,'\n',str2,'\n',str3,'\n',str4,'\n',str5,'\n')
	return #msg
end
unix.time = function()
	return os.clock()
end
unix.read = function(fd)
	return nil
end
unix.usleep = function(n_usec)
	--os.execute('sleep '..n_usec/1e6)
end

local ram_table = {
	--['id'] = 3,
	['delay'] = 5, -- Return Delay address
	['led'] = 25,
	['torque_enable'] = 24,
	['battery'] = 42, -- cannot write
	['temperature'] = 43, -- cannot write
	['hardness'] = 34,
	['velocity'] = 32,
	['command'] = 30,
	['position'] = 36, -- cannot write
}

function libDynamixel.set_ram(fd,id,addr,value)
	local inst = DynamixelPacket.write_byte(id, addr, value);
	return unix.write(fd, inst);
end

function libDynamixel.get_ram(fd,id,addr)
	local twait = 0.100;
	local inst = DynamixelPacket.read_data(id, addr, 1);
	-- TODO: Can we eliminate the read? flush?
	unix.read(fd); -- clear old status packets
	unix.write(fd, inst)
	local status = libDynamixel.get_status(fd, twait);
	if status then
		return status.parameter[1];
	end
end

function init_device_handle(obj)
	for key,addr in pairs(ram_table) do
		obj['set_'..key] = function(self,id,val)
			return libDynamixel.set_ram(self.fd,id,addr,val)
		end
		obj['get_'..key] = function(self,id)
			return libDynamixel.get_ram(self.fd,id,val,addr)
		end
	end
	return obj
end

function libDynamixel.parse_status_packet(pkt)
	local t = {};
	t.id = pkt:byte(3);
	t.length = pkt:byte(4);
	t.error = pkt:byte(5);
	t.parameter = {pkt:byte(6,t.length+3)};
	t.checksum = pkt:byte(t.length+4);
	return t;
end

function libDynamixel.close(self)
	-- fd of 0,1,2 are stdin, stdout, sterr respectively
	if fd < 3 then
		return false
	end
	-- TODO: is there a return value here for errors/success?
	unix.close( fd );
	return true
end

libDynamixel.reset = function(self)
	--print("Reseting Dynamixel tty!");
	libDynamixel.close( self.fd );
	unix.usleep( 100000) ;
	libDynamixel.open(  self.fd, self.ttyname  );
	return true
end

libDynamixel.get_status = function( self, timeout )
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
		io.write( string.format("Ping: Dynamixel ID %d\n",id) )
		libDynamixel.send_ping(fd, id);
		local status = libDynamixel.get_status(fd, twait);
		if status then
			io.write(status.id)
		end
	end
end

libDynamixel.set_id = function(fd, idOld, idNew)
	local addr = 3;  -- ID
	local inst = DynamixelPacket.write_byte(idOld, addr, idNew);
	return unix.write(fd, inst);
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

libDynamixel.sync_write_word = function(fd, ids, addr, data)
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

function libDynamixel.open( ttyname, ttybaud )
	-------------------------------
	-- Perform require upon an open
	local baud = ttybaud or 1000000;
	local fd = -1
	if ttyname and ttyname=='fake' then
		fd = -2; -- FAKE
	else
		unix = require('unix');
		stty = require('stty');
	end
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

	-------------------
	if fd~=-2 then
		fd = unix.open(ttyname, unix.O_RDWR+unix.O_NOCTTY+unix.O_NONBLOCK);
		assert(fd > 2, "Could not open port");
		-- Setup serial port parameters
		stty.raw(fd);
		stty.serial(fd);
		stty.speed(fd, baud);
	end
	-------------------

	-- Object of the Dynamixel
	local obj = {}
	obj.fd = fd
	obj.ttyname = ttyname
	obj.baud = baud
	obj = init_device_handle(obj)
	return obj;
end

return libDynamixel