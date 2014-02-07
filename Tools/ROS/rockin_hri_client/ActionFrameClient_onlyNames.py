from Tkinter import *

class Application(Frame):
   
   goto = "{\"frame\":\"motion\", \"frame_elements\":{\"goal\":\"kitchen\"}}"
   bring = "{\"frame\":\"bringing\", \"frame_elements\":{\"theme\":\"pipe\",\"goal\":\"unloading area\"}}"
   bring2 = "{\"frame\":\"bringing\", \"frame_elements\":{\"theme\":\"pipe\",\"source\":\"loading area\",\"goal\":\"unloading area\"}}"
   take = "{\"frame\":\"taking\", \"frame_elements\":{\"theme\":\"jar\"}}"
   take2 = "{\"frame\":\"taking\", \"frame_elements\":{\"theme\":\"jar\",\"source\":\"table\"}}"
   find = "{\"frame\":\"searching\", \"frame_elements\":{\"theme\":\"bottle\"}}"
   find2 = "{\"frame\":\"searching\", \"frame_elements\":{\"theme\":\"bolt\",\"ground\":\"unloading area\"}}"
   releasing = "{\"frame\":\"releasing\", \"frame_elements\":{\"theme\":\"box\"}}"	 		 
   releasing2 = "{\"frame\":\"releasing\", \"frame_elements\":{\"theme\":\"box\",\"place\":\"table\"}}"	   
   put = "{\"frame\":\"placing\", \"frame_elements\":{\"theme\":\"bolt\",\"goal\":\"unloading area\"}}"
   enter = "{\"frame\":\"entering\", \"frame_elements\":{\"goal\":\"living room\"}}"
   exit = "{\"frame\":\"exiting\", \"frame_elements\":{\"source\":\"unloading area\"}}"	
   Socket = None

   def sendCmd(self,cmd):
	print "sending %s" % cmd
	self.Socket.send(cmd)

   def createWidgets(self):
      	self.GOTO =  Button(self, command = lambda: self.sendCmd(self.goto))
	self.GOTO["text"] = "Go to the kitchen"

	self.BRING =  Button(self, command = lambda: self.sendCmd(self.bring))
	self.BRING["text"] = "Bring the pipe to the unloading area"

	self.BRING2 =  Button(self, command = lambda: self.sendCmd(self.bring2))
	self.BRING2["text"] = "Bring the pipe from the loading area to the unloading area"

	self.TAKE =  Button(self, command = lambda: self.sendCmd(self.take))
	self.TAKE["text"] = "Take the jar"

	self.TAKE2 =  Button(self, command = lambda: self.sendCmd(self.take2))
	self.TAKE2["text"] = "Take the jar on the table of the kitchen"

	self.FIND =  Button(self, command = lambda: self.sendCmd(self.find))
	self.FIND["text"] = "Find the bottle"

	self.FIND2 =  Button(self, command = lambda: self.sendCmd(self.find2))
	self.FIND2["text"] = "Find the bolt in the unloading area"

	self.RELEASE =  Button(self, command = lambda: self.sendCmd(self.releasing))
	self.RELEASE["text"] = "Release the box"

	self.RELEASE2 =  Button(self, command = lambda: self.sendCmd(self.releasing2))
	self.RELEASE2["text"] = "Release the box on the table"

	self.PUT =  Button(self, command = lambda: self.sendCmd(self.put))
	self.PUT["text"] = "Put the bolt in the unloading area"

	self.ENTER =  Button(self, command = lambda: self.sendCmd(self.enter))
	self.ENTER["text"] = "Enter the living room"

	self.EXIT =  Button(self, command = lambda: self.sendCmd(self.exit))
	self.EXIT["text"] = "Exit the unloading area"

	self.QUIT = Button(self)
      	self.QUIT["text"] = "QUIT"
      	self.QUIT["command"] = self.quit

      	#self.GOTO.pack(ipadx=60)
      	self.GOTO.pack(fill='x')
      	self.BRING.pack(fill='x')
	self.BRING2.pack(fill='x')
      	self.TAKE.pack(fill='x')
	self.TAKE2.pack(fill='x')
	self.FIND.pack(fill='x')
	self.FIND2.pack(fill='x')
	self.RELEASE.pack(fill='x')
	self.RELEASE2.pack(fill='x')	
	self.PUT.pack(fill='x')
	self.ENTER.pack(fill='x')
	self.EXIT.pack(fill='x')
	self.QUIT.pack(fill='x')

      	self.GOTO.pack({"side": "top"})
      	self.BRING.pack({"side": "top"})
      	self.BRING2.pack({"side": "top"})
      	self.TAKE.pack({"side": "top"})
      	self.TAKE2.pack({"side": "top"})
      	self.FIND.pack({"side": "top"})
      	self.FIND2.pack({"side": "top"})
      	self.RELEASE.pack({"side": "top"})
      	self.RELEASE2.pack({"side": "top"})
        self.PUT.pack({"side": "top"})
      	self.ENTER.pack({"side": "top"})
      	self.EXIT.pack({"side": "top"})
	self.QUIT.pack({"side": "top"})

   def __init__(self, master=None):
      	Frame.__init__(self, master)
      	self.pack()
      	self.createWidgets()
