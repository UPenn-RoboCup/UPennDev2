module(..., package.seeall)

motion = {}

motion.fsms = {'Manipulation'}

motion.keyframes =
  THOR_HOME..'/Data/robotis_arm/keyframes.lua'

motion.walk = {}
motion.walk.module = 'walk_osc'
motion.walk.parameters = 
  THOR_HOME..'/Data/robotis_arm/parameters_walk_osc.lua'

motion.stand = {}
motion.stand.module = nil
motion.stand.parameters = nil

motion.step = {}
motion.step.module = nil
motion.step.parameters = nil
