%function plot_occ(map)
	occ = shm('ocmOcc185Yida');
	map = occ.get_map();
	mapsize = size(map,2);
	mapsize = sqrt(mapsize);
	map = reshape(map, [mapsize, mapsize]);

	spy(map);
