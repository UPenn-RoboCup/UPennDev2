function plot_occ(map)
	mapsize = size(map,2);
	mapsize = sqrt(mapsize);
	map = reshape(map, [mapsize, mapsize]);
	map(map > 0) = 1;
	map(map < 0) = 0;
	spy(map);
