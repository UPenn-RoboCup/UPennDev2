function plot_occ(occ)
  occ_p = occ.map;
  robot_pos = occ.robot_pos();
  map_resolution = 1 / occ.mapsize;

  occ_p(occ_p < 1e-4) = 0;
  [occ_row, occ_col, occ_v] = find(occ_p);
  occ_row = occ_row * map_resolution;
  occ_col = occ_col * map_resolution;
  robot_pos = robot_pos * map_resolution;
  occ_row = occ_row - robot_pos(1);
  occ_col = robot_pos(2) - occ_col;

  hold on;
  plot(occ_row(find(occ_v >= 0.9)), occ_col(find(occ_v >= 0.9)), '*');
  plot(occ_row(find(occ_v > 0.8 & occ_v < 0.9)), occ_col(find(occ_v > 0.8 & occ_v < 0.9)), 'o');
  % draw robot body
  rectangle('Position', [-0.06, -0.04, 0.12, 0.07], ...
            'Curvature',[0.8, 0.4],...
            'LineStyle', '--');
  % draw robot head
  rectangle('Position', [-0.03, -0.03, 0.06, 0.08],...
            'Curvature', 0.7);

  hold off;
  axis([-0.5 0.5 -0.2 0.8]);
  set(gca, 'xtick', -0.5:0.1:0.5);
  set(gca, 'ytick', -0.2:0.1:0.8);
  grid on;
