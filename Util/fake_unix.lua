-- A poor man's unix library
local fake_unix = {}

fake_unix.write = function(fd,msg)
	local str = string.format('\n%s fd:(%d) sz:(%d)',type(msg),fd,#msg)
	local str2 = 'Dec:\t'
	local str3 = 'Hex:\t'
	for i=1,#msg do
		str2 = str2..string.format('%3d ', msg:byte(i))
		str3 = str3..string.format(' %.2X ', msg:byte(i))
	end
	io.write(str,'\n',str2,'\n',str3,'\n')
	return #msg
end

fake_unix.time = function()
	return os.clock()
end

fake_unix.read = function(fd)
	return nil
end

fake_unix.usleep = function(n_usec)
	--os.execute('sleep '..n_usec/1e6)
end

fake_unix.close = function( fd )
	io.write('Closed fd ',fd)
end

return fake_unix