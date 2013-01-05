addpath(genpath('.'))
startup
dcm = shm('dcm');

% test stand servo id
servo_id = 1

% init timing variables
dt = 0.005;
refresh_rate = 15;

% init circular buffers
N = 1000; 
force_buffer = zeros(1,N);
position_buffer = zeros(1,N);

% init plots
figure;
subplot(2, 1, 1);
h_force_plot = plot([1:N]*dt, force_buffer);
title('force actual')
subplot(2, 1, 2);
h_position_plot = plot([1:N]*dt, position_buffer);
title('position actual')

i = 0;
while (true)
  pause(dt);
  i = mod(i, N) + 1;

  % update shared memory
  position_command = dcm.get_joint_position();
  position_actual = dcm.get_joint_position_sensor();
  force_command = dcm.get_joint_force();
  force_actual = dcm.get_joint_force_sensor();

  % update buffers
  position_buffer(i) = position_actual(servo_id);
  force_buffer(i) = force_actual(servo_id);

  % update figures
  if (mod(i, refresh_rate) == 0)
    set(h_force_plot, 'YData', ... 
      [force_buffer(i+1:end) force_buffer(1:i)]);
    set(h_position_plot, 'YData', ... 
      [position_buffer(i+1:end) position_buffer(1:i)]);
    drawnow;
  end
end
