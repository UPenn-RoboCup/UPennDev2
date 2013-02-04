local glue = require'glue'
local ffi = require'ffi'
local bmpconv = require'bmpconv'
local readfile = glue.readfile
require'unit'
local libpng = require'libpng'


ffi = require 'ffi'
local cdata = ffi.new('unsigned char[?]', 3, {234,190,200})

print(cdata[2])

local filename = 'Image-1-10.png'
local img = libpng.load({path = filename})

