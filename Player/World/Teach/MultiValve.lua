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

local color_posterior = {}
local color_priors = {
	cyan = 1,
	magenta = 1,
	yellow = 1,
}
-- Normalize
local color_priors_sum = 0
for k,v in pairs(color_priors) do color_priors_sum = color_priors_sum + v end
for k,v in pairs(color_priors) do
	color_priors[k] = color_priors[k] / color_priors_sum
	color_posterior[k] = color_priors[k]
end

-- When human says a word, multiply by these values
local color_human_updates = {
	cyan = {cyan = 0.9, magenta = 0.3, yellow = 0.1,},
	magenta = {cyan = 0.3, magenta = 0.9, yellow = 0.1,},
	yellow = {cyan = 0.1, magenta = 0.1, yellow = 0.9,},
}

-- Should have a common API (meta/raw)
function MultiValve.send()
  local lA_raw = c_zlib(ImProc.labelA_d, ffi.sizeof(ImProc.labelA_d))
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
	}
	local tracked, t_sum = {}, 0
	for color, color_index in pairs(colors) do
		----[[
	  local color_detected = ImageProc.connected_regions(
			tonumber(ffi.cast('intptr_t', ffi.cast('void *', ImProc.labelB_d))),
			ImProc.wb,
			ImProc.hb,
			color_index
		)
		--]]
		--[[
	  local color_detected = ImageProc2.connected_regions(
			ImProc.labelB_d,
			ImProc.wb,
			ImProc.hb,
			color_index
		)
		--]]
		if color_detected then
			local bbox_stats
			for i, d in ipairs(color_detected) do
				bbox_stats = ImageProc2.color_stats(ImProc.labelB_d, ImProc.wb, ImProc.hb, color_index, d.boundingBox)
				--for k, v in pairs(bbox_stats) do d[k] = d[k] or v end
				for k, v in pairs(d) do bbox_stats[k] = v or bbox_stats[k] end
				bbox_stats.color = color
				bbox_stats.p = 1
				t_sum = t_sum + bbox_stats.p
				table.insert(tracked, bbox_stats)
			end
		end
	end
	-- Normalize tracks
	--io.write('Stage 1:')
	for i,track in ipairs(tracked) do
		track.p = track.p / t_sum
		--io.write(' ', track.p)
	end
	--io.write('\n')
	
	-- Robot only guess based on area alone
	table.sort(tracked)
	local ntracked = #tracked
	local t_sum = 0
	for i,track in ipairs(tracked) do
		--track.p = 1 / (2^i)
		--track.p = 1 / i
		track.p = track.area
		t_sum = t_sum + track.p
	end
	-- Normalize
	--io.write('Stage 2:')
	for i,track in ipairs(tracked) do
		track.p = track.p / t_sum
		--io.write(' ', track.p)
	end
	--io.write('\n')
	
	-- Add human guess to color
	local g_t = hcm.get_guidance_t()
	while g_t > 0 do
		hcm.set_guidance_t({0})
		local g_color = hcm.get_guidance_color():match('GRAB%s(%S*)%sVALVE')
		hcm.set_guidance_color('NONE')
		if not g_color then break end
		g_color = g_color:lower()
		local updates = color_human_updates[g_color]
		if not updates then break end
		-- Update the ones we are tracking
		local p_sum = 0
		for k,v in pairs(color_posterior) do
			color_posterior[k] = v * updates[k]
			p_sum = p_sum + color_posterior[k]
		end
		-- Normalize
		io.write('Color Posterior:')
		for k,v in pairs(color_posterior) do
			color_posterior[k] = v / p_sum
			io.write(' ', k, '|', color_posterior[k])
		end
		io.write('\n')
		break
	end
	
	-- Update tracked probabilities based on color posterior
	local t_sum = 0
	for i,track in ipairs(tracked) do
		track.p = track.p * color_posterior[track.color]
		t_sum = t_sum + track.p
	end
	-- Normalize
	--io.write('Stage 3:')
	for i,track in ipairs(tracked) do
		track.p = track.p / t_sum
		--io.write(' ', track.p)
	end
	--io.write('\n')
	
	-- Save
	detected.tracked = tracked
end

function MultiValve.exit()
end

return MultiValve
