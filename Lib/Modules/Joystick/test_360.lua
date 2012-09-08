require 'xbox360'
require 'unix'

xbox360.open()

unix.usleep(5e6)

xbox360.close()