function [nBytes] = update_head_camera( jimg_fd )
	%UPDATE_HEAD_CAMERA Update the omap plot
		% Return the number of network bytes
		
	global H_HCAMERA RECENT_NETWORK_USAGE TOTAL_NETWORK_USAGE;
	nBytes = 0;
	while udp_recv('getQueueSize',jimg_fd) > 0
		udp_data = udp_recv('receive',jimg_fd);
		nBytes = nBytes + numel(udp_data);
	end
	thor_camera_img = djpeg(udp_data);
	set(H_HCAMERA,'Cdata', thor_camera_img);

end