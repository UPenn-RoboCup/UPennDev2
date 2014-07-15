#!/usr/bin/env python -i

# Very important for linux, via:
# http://stackoverflow.com/questions/8361437/linker-error-lunatic-python-lua-requiresocket-undefined-symbol-lua-getme
import sys

if sys.platform != "darwin":
	import DLFCN
	sys.setdlopenflags(DLFCN.RTLD_NOW | DLFCN.RTLD_GLOBAL)

import os
# Go to the root directory for includes
CWD = os.getcwd()
HOME = CWD.replace("/ROS","")
os.chdir(HOME)
	
# Import luantic-python
sys.path.append(HOME+"/Modules/python")
import lua
# Have all the framework files/Modules available
lua.execute('dofile"fiddle.lua"')
# Have the globals if needed...
lg = lua.globals()
require = lua.require

# Go back to the current directory
os.chdir(CWD)
