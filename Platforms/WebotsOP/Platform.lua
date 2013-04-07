
local Platform = {}
Platform.path = {}
Platform.path[#Platform.path+1] = '/?.lua;'
Platform.path[#Platform.path+1] = '/Body/?.lua;'
Platform.path[#Platform.path+1] = '/Camera/?.lua;'
Platform.path[#Platform.path+1] = '/Kinematics/?.lua;'

Platform.cpath = {}
Platform.cpath[#Platform.cpath+1] = '/?.so;'
Platform.cpath[#Platform.cpath+1] = '/Body/?.so;'
Platform.cpath[#Platform.cpath+1] = '/Camera/?.so;'
Platform.cpath[#Platform.cpath+1] = '/Kinematics/?.so;'

return Platform

