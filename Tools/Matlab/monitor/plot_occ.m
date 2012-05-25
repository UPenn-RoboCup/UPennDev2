function plot_occ(occ)
  robotsizex = 5;
  robotsizey = 3;
  hold on;
  set(gca,'YDir','reverse');
  imagesc(occ.map);
  colormap(gray);
  rectangle('Position',[occ.robot_pos(1)-robotsizex,occ.robot_pos(2)-robotsizey,...
                        robotsizex*2, robotsizey*2]);
%  axis([-0.5 0.5 -0.2 0.8]);
  axis([0 50 0 50]);
  set(gca, 'xtick', -0.5:0.1:0.5);
  set(gca, 'ytick', -0.2:0.1:0.8);
  hold off;
