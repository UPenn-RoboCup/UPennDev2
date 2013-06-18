function [ nBytes ] = update_omap( omap_fd )
	%UPDATE_OMAP Update the omap plot
		% Return the number of network bytes
		
	global H_OCCUPANCY;
	nBytes = 0;
	while udp_recv('getQueueSize',omap_fd) > 0
		udp_data = udp_recv('receive',omap_fd);
		nBytes = nBytes+numel(udp_data);
	end




return;




	
	% Grab the data
  pose_data = msgpack('unpack',udp_data);
  pose_a = double(pose_data.a);
  pose_x = double(pose_data.x);
  pose_y = double(pose_data.y);
  new_omap_xmin = double(pose_data.omapXmin);
  new_omap_ymin = double(pose_data.omapYmin);
	
	% TODO: omap_xmin, omap_ymin, omap_invRes, should be global, or with omap
	% TODO: They could just be zerod out in plotting, which could make things easy
  sa = sin(pose_a);
  ca = cos(pose_a);
  xi = (pose_x - omap_xmin) * omap_invRes;
  yi = (pose_y - omap_ymin) * omap_invRes;
	
	% Format for the icon
	% TODO: scale should be related to the axes, which should scale the image down
	icon_scale = floor(sqrt(omap_w*omap_h) / 10);
	icon_xs = [.125 -.125 -.125] * icon_scale;
	icon_ys = [0 -.10 +.10] * icon_scale;
  xr = icon_xs*ca - icon_ys * sa + xi;
  yr = icon_xs*sa + icon_ys * ca + yi;
	% Set the data
  set(h_pose,'XData',xr);
  set(h_pose,'YData',yr);
	
	% Return the number of network bytes
	nBytes = numel(udp_data);
end
