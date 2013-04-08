module(..., package.seeall)

world_manager = {
  physics_time_step = 0.000334,
  time_channel_endpoint = 'tcp://127.0.0.1:12000',
  time_channel_rate = 1000,
}

comms_manager = {
  -- # joints controlled by comms_manager
  joint_max = 28,
  -- joint cfm damping
  joint_damping = 1,
  -- impedance controller settings
  d_break_freq = 100,
  p_gain_constant = 1600,
  i_gain_constant = 1600,
  d_gain_constant = 1600,
  -- initial controller gains
  p_gain_default = 0.8,
  i_gain_default = 0.1,
  d_gain_default = 0.005,
}
