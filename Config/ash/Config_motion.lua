module(..., package.seeall)

motion = {}

motion.fsms = {'Locomotion'}

motion.keyframes =
  THOR_HOME..'/Data/ash/keyframes.lua'

motion.walk = {}
motion.walk.module = 'walk_osc'
motion.walk.parameters = 
  THOR_HOME..'/Data/ash/parameters_walk_osc.lua'

motion.stand = {}
motion.stand.module = 'stand_osc'
motion.stand.parameters = 
  THOR_HOME..'/Data/ash/parameters_stand_osc.lua'

motion.step = {}
motion.step.module = nil
motion.step.parameters = nil
