% image_array is a matlab struct that can be used to reconstruct 
% images (yuyv, label) broadcast by monitor

function h = image_array(name)
  h.name = name;
	h.division = 1;
	h.section = 1;
	h.section_fill = [];
	h.image_buf = uint8(0);
	h.image = uint8(0);
	h.width = 0;
	h.height = 0;
	h.updated = 0;
	h.count = 0;
  h.update = @update;
	h.reset = @reset;
	h.get_update = @get_update;
	h.get_width = @get_width;
	h.get_height = @get_height;

	function width = get_width()
		width = h.width;
	end

	function height = get_height()
		height = h.height;
	end

	function [updated,count] = get_update()
		updated = h.updated;
		count = h.count;
	end

	function ret = reset()
			h.section_full = zeros(1,h.division);
			h.image_buff = uint8(0);
			h.updated = 0;
			h.count = 0;
			ret = 1;
	end

	function image = update(packet)
%	fprintf('type: %d, name %d\n',packet.type,h.name)
		if (packet.type == h.name) 

			% initiate
			if (size(h.image_buf,2)==1)
				disp('Initiate');
				h.width = packet.width;
				h.height = packet.height;
				h.image_buff = uint8(0);
				h.division = packet.division;
				h.section_full = zeros(1,h.division);
			end
				startpnt = 1 + (packet.section-1)*size(packet.data,2);
				h.image_buf(startpnt:startpnt+size(packet.data,2)-1) = packet.data;
				h.section_full(packet.section) = 1;
			% check 
			if (sum(h.section_full) == h.division)
				h.image = h.image_buf;
				h.updated = 1;
				h.count = h.count + 1;
			end	
%fprintf('Updating %d\n',h.name)
%disp(h.section_full);
		end
		image = h.image;
  end

end
