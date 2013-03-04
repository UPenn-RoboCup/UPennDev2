module(..., package.seeall)

motion = {}

motion.keyframes =
  _HOME_..'/Data/ash/keyframes.lua'

motion.walk = {}
motion.walk.module = 'walk_osc'
motion.walk.parameters = 
  _HOME_..'/Data/ash/parameters_walk_osc.lua'

motion.stand = {}
motion.stand.module = nil
motion.stand.parameters = nil

motion.step = {}
motion.step.module = nil
motion.step.parameters = nil
