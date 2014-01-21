# Import luantic-python
import lua
# Have all the framework files/Modules available
lua.execute('dofile"riddle.lua"')
# Have the globals if needed...
lg = lua.globals()
require = lua.require
