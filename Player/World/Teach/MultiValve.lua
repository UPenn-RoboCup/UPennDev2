local MultiValve = {}
require'hcm'
local ImageProc2 = require'ImageProc.ffi2'
local c_zlib = require'zlib.ffi'.compress
local ImProc, colors
local lA_meta = {
  c = 'zlib',
  id = 'labelA',
}
local detected
--local ptable = require'util'.ptable

-- Should have a common API (meta/raw)
function MultiValve.send()
  local lA_raw = c_zlib(ImProc.labelA_d, ImProc.labelA_n)
  lA_meta.sz = #lA_raw
  return {
  	{lA_meta, lA_raw},
		{detected},
	}
end

function MultiValve.entry(cfg)
	colors = cfg.vision.colors
	ImProc = ImageProc2.new(cfg.w, cfg.h, cfg.vision.scaleA, cfg.vision.scaleB)
  lA_meta.w = ImProc.wa
  lA_meta.h = ImProc.ha
	ImageProc2.load_lut(ImProc, table.concat{HOME, "/Data/", "lut_", cfg.lut, ".raw"})
end

function MultiValve.update(rgb, depth)
  ImageProc2.rgb_to_labelA(ImProc, rgb.data)
  ImageProc2.block_bitor(ImProc)
	-- Nothing detected yet
	detected = {
		id = 'detect',
		n = 0,
	}
	local color_detected
	for color, color_index in pairs(colors) do
	  color_detected = ImageProc2.connected_regions(
			tonumber(ffi.cast('intptr_t', ffi.cast('void *', ImProc.labelB_d))),
			ImProc.wb,
			ImProc.hb,
			color_index
		)
		if color_detected then
			detected[color] = color_detected
			detected.n = detected.n + #color_detected
			local bbox_stats
			for i, d in ipairs(color_detected) do
				bbox_stats = ImageProc2.color_stats(ImProc, 'b', color_index, d.boundingBox)
				for k, v in pairs(bbox_stats) do d[k] = d[k] or v end
			end
		end
	end
	-- Probabilities
	--[[
	local uniform_probability = 1 / detected.n
	-- Check any events
	local g_color = hcm.get_guidance_color()
	local g_t = hcm.get_guidance_t()
	if g_t > 0 then
		-- Set to uniform
		for color, color_index in pairs(colors) do if detected[color] then detected[color].probability = uniform_probability end end
		return
	end
	local update = {}
	for color, color_index in pairs(colors) do
		update[color] = 1
		if g_color:lower()==color then
			update[color] = 0.9 * update[color]
		else
			update[color] = 0.1 * update[color]
		end
		if detected[color] then detected[color].probability = uniform_probability * update[color] end
	end
	hcm.set_guidance_t({0})
	--]]
end

function MultiValve.exit()
end

return MultiValve
