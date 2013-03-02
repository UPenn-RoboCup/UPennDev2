-- prefix is defined by vrep_launch

if simGetScriptExecutionCount() == 0 then
	simSetStringSignal('controller_include_prefix', prefix)
end
