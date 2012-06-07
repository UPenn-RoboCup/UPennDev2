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

  occ_odom = occ.odom;
  % Robot Triangle
  odom_x = occ_odom(1);
  odom_y = occ_odom(2);
  odom_a = occ_odom(3);
  angle1 = pi/2 + odom_a;
  angle2 = pi + pi/4 + odom_a;
  angle3 = -pi/4 + odom_a;
  len1 = 1;
  len2 = 0.8;
  len3 = len2;
  tri.v(:,1) = [len1, len2, len3] .* cos([angle1, angle2, angle3]);
  tri.v(:,2) = [len1, len2, len3] .* sin([angle1, angle2, angle3]);
  tri.v = tri.v * 0.04;
  tri.v(:,1) = tri.v(:,1) - odom_y;
  tri.v(:,2) = tri.v(:,2) + odom_x;

  hold on;
  patch(tri.v(:,1), tri.v(:,2),'b');
  plot(occ_row(find(occ_v >= 0.95)), occ_col(find(occ_v >= 0.95)), '*');
  plot(occ_row(find(occ_v > 0.8 & occ_v < 0.95)), ...
        occ_col(find(occ_v > 0.8 & occ_v < 0.95)), 'o');
  % draw robot body
  hB = rectangle('Position', [-0.06, -0.04, 0.12, 0.07], ...
            'Curvature',[0.8, 0.4],...
            'LineStyle', '--');
  % draw robot head
  hH = rectangle('Position', [-0.03, -0.03, 0.06, 0.08],...
            'Curvature', 0.7);
  zdir = [0 0 1];
  rotate(hB, zdir, 0.25 * pi);
  rotate(hH, zdir, 0.25 * pi);

  hold off;
  axis([odom_y-0.5 odom_y+0.5 odom_x-0.2 odom_x+0.8]);
  set(gca, 'xtick', -0.5:0.1:0.5);
  set(gca, 'ytick', -0.2:0.1:0.8);
  grid on;
