module(..., package.seeall)

comms_manager = {}

comms_manager = {
  -- # joints controlled by comms_manager
  joint_max = 29,
  -- impedance controller settings
  p_gain_constant = 1000,
  i_gain_constant = 1000,
  d_gain_constant = 1000,
  d_break_freq = 30,
  -- initial controller gains
  p_gain_default = 0.5,
  i_gain_default = 0.1,
  d_gain_default = 0.005,
}
