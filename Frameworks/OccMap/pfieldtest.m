occ_p = occ.map;
robot_pos = occ.robot_pos();
map_resolution = 1 / occ.mapsize;

occ_p(occ_p < 1e-4) = 0;
[occ_r, occ_c, occ_v] = find(occ_p);
occ_r = occ_r - robot_pos(1);
occ_c = robot_pos(2) - occ_c;
occ_row = occ_r * map_resolution;
occ_col = occ_c * map_resolution;

pvector = zeros(6, 2500);
for  i_c = 1 : 50 
  for j_r = 1 : 50
    c = robot_pos(2) - i_c; %- 25;
    r = j_r - robot_pos(1); % - 10;
    closeOb = [10000,10000,10000,1000, 0, 0];
%    dist = map_resolution * sqrt(c^2 + r^2);
%    angle = atan2(r, c);
%    closeOb = [r, c, dist, angle, dist*sin(angle), dist*cos(angle)];

    for k = 1 : size(occ_r, 1)
      dist = map_resolution * sqrt((r-occ_r(k))^2+(c-occ_c(k))^2);
      angle = atan2(r-occ_r(k), c-occ_c(k));
      if dist < closeOb(3)
        closeOb = [r, c, dist, angle, sin(angle)/dist, cos(angle)/dist];
      end
    end

    pvector(:, (j_r-1)*50 + i_c) = closeOb';
  end
end

robot_pos = robot_pos * map_resolution;
%occ_row = occ_row - robot_pos(1);
%occ_col = robot_pos(2) - occ_col;

pvector(1, :) = pvector(1, :) * map_resolution;
%%pvector(1, :) = pvector(1, :) * map_resolution - robot_pos(1);
pvector(2, :) = pvector(2, :) * map_resolution;
%%pvector(2, :) = robot_pos(2) - pvector(2, :) * map_resolution;

reject_gain = 1;
pvector(5, :) = pvector(5, :) * reject_gain;
pvector(6, :) = pvector(6, :) * reject_gain;

hold on;
plot(occ_row, occ_col, '*');
%plot(occ_row(find(occ_v >= 0.9)), occ_col(find(occ_v >= 0.9)), '*');
%plot(occ_row(find(occ_v > 0.8 & occ_v < 0.9)), occ_col(find(occ_v > 0.8 & occ_v < 0.9)), 'o');
quiver(pvector(1,:), pvector(2,:), pvector(5,:), pvector(6,:), 2);

% -----
odom_x = 0.1;
odom_y = 0.05;
odom_a = pi/8;
angle1 = pi/2 + odom_a;
angle2 = pi + pi/4 + odom_a;
angle3 = -pi/4 + odom_a;
len1 = 1;
len2 = 0.8;
len3 = len2;
tri.v(:,1) = [len1, len2, len3] .* cos([angle1, angle2, angle3]);
tri.v(:,2) = [len1, len2, len3] .* sin([angle1, angle2, angle3]);
tri.v = tri.v * 0.04;
tri.v(:,1) = tri.v(:,1) + odom_y;
tri.v(:,2) = tri.v(:,2) + odom_x;
patch(tri.v(:,1), tri.v(:,2),'b');

% -----
% draw robot body
% rectangle('Position', [-0.06, -0.04, 0.12, 0.07],'Curvature',[0.8, 0.4], 'LineStyle', '--');
% draw robot head
% rectangle('Position', [-0.03, -0.03, 0.06, 0.08],'Curvature', 0.7);

hold off;
axis([-0.5 0.5 -0.2 0.8]);
set(gca, 'xtick', -0.5:0.1:0.5);
set(gca, 'ytick', -0.2:0.1:0.8);
grid on;
