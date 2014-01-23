#!/usr/bin/env python
 
import socket
from Tkinter import *
from ActionFrameClient import *
  
TCP_IP = '127.0.0.1'
TCP_PORT = 5004
BUFFER_SIZE = 1024

def connect(ip,port):
	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s.connect((ip, port))
	return s

#command = "{\"frame\":\"motion\", \"frame_elements\":{\"goal\":\"to the kitchen\", \"source\":\"from the living room\"}}"
#command = raw_input("Insert a command:")
#while command != "END":
#s.send(command)

#data = s.recv(BUFFER_SIZE)
#print "received data:", data
#command = raw_input("Insert a command:")

#s.close()
 
if __name__ == '__main__':
	sock = connect(TCP_IP,TCP_PORT)
	root = Tk()
	app = Application(master=root)	
	app.Socket = sock
	app.mainloop()
	root.destroy()
	sock.close()
