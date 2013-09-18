local tcp = require 'tcp'
local unix = require 'unix'

fd = tcp.open('192.168.0.10', 10940, 1)
print(fd)

A = 'VV;\n'

unix.write(fd, A);
res = unix.read(fd, 1024);
print(res)

