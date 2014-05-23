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
local tou_ch = simple_ipc.new_subscriber(Config.human.touch.ch)
local tou_che = simple_ipc.new_publisher(Config.human.touch.bbox_ch)
local libTouch = require'libTouch'
-- Allow logging
local DO_LOG, libLog, logger = false
if DO_LOG then
	libLog = require'libLog'
	-- Make the logger
	logger = libLog.new'touch'
end

-- Save some variables for bbox formation
local has_new_bbox = true
local x0, y0, x1, y1
local bb_major, bb_minor, bb_angle, bb_xc, bb_yc
local function form_bbox (contact)
	-- If the start of a contact, then this is the bottom/top
	if contact.e=='start' then
		x0 = contact.x
		y0 = contact.y
	elseif contact.e=='finish' then
		-- Assume tick-tock for the swipes
		-- First swipe is the major axis, second is the minor axis
		x1 = contact.x
		y1 = contact.y
		local x_diff, y_diff = x1-x0, y1-y0
		local len = math.sqrt( x_diff^2 + y_diff^2 )
		if has_new_bbox then
			-- Was the major axis
			bb_angle = math.atan2(y_diff, x_diff)
			--bb_angle = math.atan(y_diff / x_diff)
			bb_major = len
		else
			-- New bbox on this swipe
			bb_minor = len
			bb_xc = (x0 + x1) / 2
			bb_yc = (y0 + y1) / 2
		end
		-- Update for the next swipe
		has_new_bbox = not has_new_bbox
		if has_new_bbox then
			local bbox, dir
			if math.abs(bb_angle)<45*DEG_TO_RAD or math.abs(bb_angle)>135*DEG_TO_RAD then
				print('Horizontal')
				dir = 'h'
				bbox = {
					(bb_xc - bb_major / 2), -- x0
					(bb_xc + bb_major / 2), -- x1
					(bb_yc - bb_minor / 2), -- y0
					(bb_yc + bb_minor / 2), -- y1
				}
			else
				print('Vertical')
				dir = 'v'
				bbox = {
					(bb_xc - bb_minor / 2), -- x0
					(bb_xc + bb_minor / 2), -- x1
					(bb_yc - bb_major / 2), -- y0
					(bb_yc + bb_major / 2), -- y1
				}
			end
			print('touch bbox', bb_major, bb_minor, RAD_TO_DEG * bb_angle, bb_xc, bb_yc)
			print('bbox',unpack(bbox))
			-- Send to JavaScript to communicate that we have calculated just fine
			tou_che:send(mp.pack({
				id = 'bbox',
				xc = bb_xc,
				yc = bb_yc,
				a = bb_angle,
				major = bb_major,
				minor = bb_minor,
				-- Use the standard notation for non tilted, as fallback
				bbox = bbox,
				-- Direction for Kernel and Radon use
				dir = dir,
			}))
		end
	end
end

-- Callback for processing
-- Only get the full swipe information
function tou_ch.callback (s)
	local t = unix.time()
	local data = tou_ch:receive()
	for _, d in ipairs(data) do
		local evt = mp.unpack(d)
		-- evt should be an array of all full swipes
		if DO_LOG then
			evt.TIMESTAMP = t
			logger:record(evt)
		end
		-- Process and send back the bounding box
		libTouch.update(evt, form_bbox)
	end
end

local signal = require'signal'
function shutdown ()
	-- Stop the poller
	poller:stop()
	-- Save the logs
	if DO_LOG then logger:stop() end
end
signal.signal("SIGINT", shutdown)
signal.signal("SIGTERM", shutdown)

-- Start listening with the poller
poller = simple_ipc.wait_on_channels{tou_ch}
poller:start()
