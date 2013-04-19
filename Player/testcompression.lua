require 'init'
local Z = require 'Z'
local snappy = require 'snappy'
local unix = require 'unix'

local file = io.open('../../Dropbox/data/150213185940.20/estimate1-04.12.2013.13.58.10-0', 'r')
local file_str = file:read('*a')

print(#file_str)
local t0 = unix.time()
print(#Z.compress(file_str, #file_str))
print(unix.time() - t0)
t0 = unix.time()
print(#snappy.compress(file_str))
print(unix.time() - t0)
