function monitor_cop
  addpath(genpath('..'))
  startup
  pcm = shm('pcm');
  mcm = shm('mcm');

  % define foot geometry
  foot_width = 0.125;
  foot_length = 0.23;
  sole_x_offset = 0.0213876;
  sole_y_offset = 0;

  % define plot variables
  N_trace = 20;
  desired_cop_trace = zeros(2, N_trace);
  actual_cop_trace = zeros(2, N_trace);
  l_foot_pose = zeros(3, 1);
  r_foot_pose = zeros(3, 1);

  % init timing variables
  dt = 0.001;
  refresh_rate = 4;

  % define plotting functions
  function h_cop = plot_cop(cop_trace, options)
    h_cop = plot(cop_trace(1, :), cop_trace(2, :), options, 'LineWidth', 2.5);
  end

  function h_foot = plot_foot(pose)
    x_sole = pose(1) + sole_x_offset - foot_length/2;
    y_sole = pose(2) + sole_y_offset - foot_width/2;
    h_foot = rectangle('Position', [x_sole, y_sole, foot_length, foot_width]);
  end

  function update_cop(h_cop, cop_trace, color)
    set(h_cop, 'Color', color);
    set(h_cop, 'XData', cop_trace(1, :));
    set(h_cop, 'YData', cop_trace(2, :));
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
  h_actual_cop = plot_cop(actual_cop_trace, '-b');
  h_desired_cop = plot_cop(desired_cop_trace, '-k');
  h_l_foot = plot_foot(l_foot_pose);
  h_r_foot = plot_foot(r_foot_pose);
  axis([-0.4 0.4 -0.4 0.4]);

  i = 0;
  while (true)
    i = mod(i, N_trace) + 1; 
    pause(dt);

    l_foot_pose = pcm.get_l_foot_pose();
    r_foot_pose = pcm.get_r_foot_pose();
    actual_cop = pcm.get_cop();
    desired_cop = mcm.get_desired_cop();
    tipping_status = pcm.get_tipping_status();

    if (tipping_status(1) == 1)
      cop_color = 'r';
    else
      cop_color = 'b';
    end

    actual_cop_trace(1, i) = actual_cop(1);
    actual_cop_trace(2, i) = actual_cop(2);
    desired_cop_trace(1, i) = desired_cop(1);
    desired_cop_trace(2, i) = desired_cop(2);

    if (mod(i, refresh_rate) == 0)
      update_cop(h_actual_cop, ...
        [actual_cop_trace(:, i + 1:end) actual_cop_trace(:, 1:i)], ...
        cop_color ...
      );
      update_cop(h_desired_cop, ...
        [desired_cop_trace(:, i + 1:end) desired_cop_trace(:, 1:i)], ...
        'k' ...
      );
      update_foot(h_l_foot, l_foot_pose);
      update_foot(h_r_foot, r_foot_pose);
      drawnow;
    end
  end
end
