function monitor_cop
  addpath(genpath('.'))
  startup
  pcm = shm('pcm');
  mcm = shm('mcm');

  % define foot geometry
  foot_width = 0.125;
  foot_length = 0.23;
  sole_x_offset = 0.0213876;
  sole_y_offset = 0;

  % init timing variables
  dt = 0.001;
  refresh_rate = 1;

  % define plotting functions
  function h_cop = plot_cop(cop, options)
    h_cop = plot([cop(1)], [cop(2)], options, 'LineWidth', 3);
  end

  function h_foot = plot_foot(pose)
    x_sole = pose(1) + sole_x_offset - foot_length/2;
    y_sole = pose(2) + sole_y_offset - foot_width/2;
    h_foot = rectangle('Position', [x_sole, y_sole, foot_length, foot_width]);
  end

  function update_cop(h_cop, cop)
    set(h_cop, 'XData', [cop(1)]);
    set(h_cop, 'YData', [cop(2)]);
  end

  function update_foot(h_foot, pose)
    x_sole = pose(1) + sole_x_offset - foot_length/2;
    y_sole = pose(2) + sole_y_offset - foot_width/2;
    set(h_foot, 'Position', [x_sole, y_sole, foot_length, foot_width]);
  end

  % initialize plot
  figure;
  hold on;
  title('cop trajectory');
  h_actual_cop = plot_cop([0 0], 'ob');
  h_desired_cop = plot_cop([0 0], 'or');
  h_l_foot = plot_foot([0 0 0]);
  h_r_foot = plot_foot([0 0 0]); 
  axis([-0.4 0.4 -0.4 0.4]);

  i = 0;
  while (true)
    i = i + 1; 
    pause(dt);

    l_foot_pose = pcm.get_l_foot_pose();
    r_foot_pose = pcm.get_r_foot_pose();
    actual_cop = pcm.get_cop();
    desired_cop = mcm.get_desired_cop();

    if (mod(i, refresh_rate) == 0)
      update_cop(h_actual_cop, actual_cop);
      update_cop(h_desired_cop, desired_cop);
      update_foot(h_l_foot, l_foot_pose);
      update_foot(h_r_foot, r_foot_pose);
      drawnow;
    end
  end
end
