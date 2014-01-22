# Very important for linux, via:
# http://stackoverflow.com/questions/8361437/linker-error-lunatic-python-lua-requiresocket-undefined-symbol-lua-getme
import sys

if sys.platform != "darwin":
	import DLFCN
	sys.setdlopenflags(DLFCN.RTLD_NOW | DLFCN.RTLD_GLOBAL)

# Import luantic-python
import lua
# Have all the framework files/Modules available
lua.execute('dofile"fiddle.lua"')
# Have the globals if needed...
lg = lua.globals()
require = lua.require
