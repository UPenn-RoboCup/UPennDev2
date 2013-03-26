module(..., package.seeall)

comms_manager = {}

comms_manager = {
  -- # joints controlled by comms_manager
  joint_max = 12,
  -- impedance controller settings
  p_gain_constant = 1750,
  i_gain_constant = 1750,
  d_gain_constant = 1750,
  d_break_freq = 50,
  -- initial controller gains
  p_gain_default = 0.8,
  i_gain_default = 0.0,
  d_gain_default = 0.01,
}
