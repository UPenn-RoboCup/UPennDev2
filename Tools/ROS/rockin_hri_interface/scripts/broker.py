#!/usr/bin/env python
import rospy
import socket
import json
from rockin_hri_interface.msg import *

TCP_IP = '127.0.0.1'
TCP_PORT = 5004
BUFFER_SIZE = 1024

def broker():
  #setting up the node
  pub = rospy.Publisher('semantic_frames',frame)
  rospy.init_node('semantic_frame_broker')

  #setting up the connection
  s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  s.bind((TCP_IP, TCP_PORT))
  s.listen(1)

  conn, addr = s.accept()
  print 'Connection address:', addr

  while not rospy.is_shutdown(): 
    data = conn.recv(BUFFER_SIZE)
    print "received data:", data
    if not data:
      break
    f = parse_frame(data)
    pub.publish(f)
  
  conn.close()
  
def parse_frame(data):
  #a = json.loads(data)
  #json_frame = json.loads(a)
  json_frame = json.loads(data)
  
  f = frame()
  f.name = json_frame["frame"]
  print "frame: %s" % f.name
  #f.lexical_unit = json_frame["lu"]
  fes = [] #initializing frame elements array
  
  print "frame elements:"
  
  for fe in json_frame["frame_elements"]:
    a = str(fe)
    frame_e = frame_element()
    frame_e.name = a
    frame_e.role_filler = json_frame["frame_elements"][a]
    print "%s: %s" % (a, json_frame["frame_elements"][a])
    fes.append(frame_e)
  
  f.frame_elements = fes
  return f
    
if __name__ == '__main__':
  try:
    broker()
  except rospy.ROSInterruptException:
    pass
