% image_array is a matlab struct that can be used to reconstruct 
% images (yuyv, label) broadcast by monitor

function h = image_array(name)
  h.name = name;
	h.division = 1;
	h.section = 1;
	h.section_fill = [];
	h.image_buf = [];
	h.image = [];
	h.width = 0;
	h.height = 0;
  h.update = @update;
	h.get_width = @get_width;
	h.get_height = @get_height;

	function width = get_width()
		width = h.width;
	end

	function height = get_height()
		height = h.height;
	end

	function image = update(packet)
		if (packet.type == h.name) 
			% initiate
			if (isempty(h.image_buf)==1)
				h.width = packet.width;
				h.height = packet.height;
				h.image_buff = zeros(1,packet.width * packet.height);
				h.division = packet.division;
				h.section_full = zeros(h.division,1);
			end
			startpnt = 1 + (packet.section-1)*size(packet.data,2);
			h.image_buf(startpnt:startpnt+numel(packet.data)-1) = packet.data;
			h.section_full(packet.section) = 1;
			% check 
			if (sum(h.section_full) == h.division)
%				disp('full image');
%				disp([h.width,h.height]);
				h.image = h.image_buf;
				h.section_full = zeros(h.division,1);
				h.image_buff = uint8(zeros(packet.width * packet.height,1));
			end	
		end
		image = h.image;    
  end

end
