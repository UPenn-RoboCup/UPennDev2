-- copy paste this code into the vrep child script for the robot model

if (simGetScriptExecutionCount() == 0) then
  THOR_HOME = os.getenv('THOR_HOME')
  dofile(THOR_HOME..'/Run/include.lua')
  require('vrep_comms_manager')
  vrep_comms_manager.entry()
end

simHandleChildScript(sim_handle_all_except_explicit)

vrep_comms_manager.update()

if (simGetSimulationState() == sim_simulation_advancing_lastbeforestop) then
  vrep_comms_manager.exit()
end
