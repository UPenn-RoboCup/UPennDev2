-- prefix is defined by vrep_launch

if simGetScriptExecutionCount() == 0 then
  console = simAuxiliaryConsoleOpen("Aux Console", 500, 0x10)
  oldprint = print
  print = function(...)
    simAuxiliaryConsolePrint(console, ...)
  end
  
  dofile(prefix..'/Run/include.lua')
  
  require('VRepCommsManager')
  VRepCommsManager.entry()
end

VRepCommsManager.update()
