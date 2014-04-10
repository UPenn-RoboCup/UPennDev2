----------------------------------------
-- Touchscreen Motion processor
-- Listens to touch messages from ZeroMQ
-- (c) Stephen McGill, 2013
----------------------------------------
dofile'include.lua'
-- Libraries
local unix = require'unix'
local util = require'util'
local mp = require'msgpack'
local simple_ipc, poller = require'simple_ipc'
local tou_ch = simple_ipc.new_publisher'touch'
