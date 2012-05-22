function plot_occ(occ)
  robotsizex = 5;
  robotsizey = 3;
  hold on;
  set(gca,'YDir','reverse');
  imagesc(occ.map);
  rectangle('Position',[occ.robot_pos(1)-robotsizex,occ.robot_pos(2)-robotsizey,...
                        robotsizex*2, robotsizey*2]);
  axis([0 50 0 50]);
  hold off;
