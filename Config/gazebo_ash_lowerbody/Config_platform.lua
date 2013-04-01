module(..., package.seeall)

comms_manager = {}

comms_manager = {
  -- # joints controlled by comms_manager
  joint_max = 12,
  -- # joint cfm damping
  joint_damping = 1,
  -- impedance controller settings
  p_gain_constant = 1600,
  i_gain_constant = 1600,
  d_gain_constant = 1600,
  d_break_freq = 100,
  -- initial controller gains
  p_gain_default = 0.8,
  i_gain_default = 0.1,
  d_gain_default = 0.005,
}
