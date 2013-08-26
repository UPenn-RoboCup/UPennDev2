function [ nBytes ] = update_sim_omap( omap_data )

	%UPDATE_OMAP Update the omap plot
	% Return the number of network bytes
		
    
	global H_SLAM;

    thor_omap = reshape(omap_data, 401,401);
	set(H_SLAM,'Cdata', thor_omap);
   
	% Return the number of network bytes
    nBytes = 0;
    
end
