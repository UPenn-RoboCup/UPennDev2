module(..., package.seeall)

comms_manager = {}

comms_manager = {
  -- # joints controlled by comms_manager
  joint_max = 27,
  -- impedance controller settings
  p_gain_constant = 1700,
  i_gain_constant = 1700,
  d_gain_constant = 1700,
  d_break_freq = 35,
  -- initial controller gains
  p_gain_default = 0.5,
  i_gain_default = 0.0,
  d_gain_default = 0.005,
}
