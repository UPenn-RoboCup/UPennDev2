function plot_occ(occ)
  robotsizex = 5;
  robotsizey = 3;
  hold on;
	imagesc(occ.map);
  rectangle('Position',[occ.centroid(1)-robotsizex,occ.centroid(2)-robotsizey,...
                        robotsizex*2, robotsizey*2]);
  axis([0 50 0 50]);
  hold off;
