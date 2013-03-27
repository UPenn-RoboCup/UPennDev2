
init=package.loadlib("../../Lib/manip/libmanipulation_controller.so","luaopen_Manipulation_Controller")

assert(init())

Manipulation_Controller.THOR_MC_initialize()

