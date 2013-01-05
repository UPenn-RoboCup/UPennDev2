function monitor_cop
  addpath(genpath('.'))
  startup
  pcm = shm('pcm');

  % define foot geometry
  foot_width = 0.125;
  foot_length = 0.23;
  sole_x_offset = 0.0213876;
  sole_y_offset = 0;

  % init timing variables
  dt = 0.001;
  refresh_rate = 5;

  % init circular buffers
  N = 15; 
  cop_trace = zeros(2, N);

  % initialize plot
  figure;
  title('cop trajectory');
  h_cop = plot(cop_trace(1, :), cop_trace(2, :), '-b', 'LineWidth', 3);
  h_l_foot = rectangle('Position', [0, 0, foot_length, foot_width]);
  h_r_foot = rectangle('Position', [0, 0, foot_length, foot_width]);
  axis([-0.4 0.4 -0.4 0.4]);
  hold on;

  function update_cop(h_cop, cop_trace)
    set(h_cop, 'XData', cop_trace(1, :));
    set(h_cop, 'YData', cop_trace(2, :));
  end

  function update_foot(h_foot, pose)
    x_sole = pose(1) + sole_x_offset - foot_length/2;
    y_sole = pose(2) + sole_y_offset - foot_width/2;
    set(h_foot, 'Position', [x_sole, y_sole, foot_length, foot_width]);
  end

  i = 0;
  while (true)
    pause(dt);
    i = mod(i, N) + 1;

    l_foot_pose = pcm.get_l_foot_pose();
    r_foot_pose = pcm.get_r_foot_pose();
    cop = pcm.get_cop();

    cop_trace(1, i) = cop(1);
    cop_trace(2, i) = cop(2);

    if (mod(i, refresh_rate) == 0)
      update_cop(h_cop, [cop_trace(:, i+1:end) cop_trace(:, 1:i)]);
      update_foot(h_l_foot, l_foot_pose);
      update_foot(h_r_foot, r_foot_pose);
      drawnow;
    end
  end
end
