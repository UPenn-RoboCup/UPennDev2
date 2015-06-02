#!/usr/local/bin/luajit
dofile('../UPennDev/include.lua')
local mp = require'msgpack'
local x = mp.pack(Config)
io.write(x)

