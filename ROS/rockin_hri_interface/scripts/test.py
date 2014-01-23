#!/usr/bin/env python
import rospy
from rockin_hri_interface.msg import *
#from PrologInterface.srv import *

def callback(data):
    print "SEMANTIC FRAME PUBLISHED:"
    print "name: %s" % data.name
    #print "lexical_unit: %s" % data.lexical_unit
    for fe in data.frame_elements:
      print "%s: %s" % (fe.name, fe.role_filler)
    
#    srv = prologSrv()
     
#    if data.name == "motion":
#          for fe in data.frame_elements:
#    	    fe.name == "goal":
#	      srv.request.predicate = getPredicate(fe.role_filler)
#	      arguments = getArguments(fe.role_filler)
#	      for argument in arguments:
#		srv.request.arg.append(argument)

queryTokenRep = {}

def initializeQueryTokenRep():
  queryTokenRep["to"] = "location"
  queryTokenRep["in"] = "location"
  queryTokenRep["for"] = "location"
  queryTokenRep["at"] = "location"
  queryTokenRep["on"] = "location"
  queryTokenRep["into"] = "location"
  queryTokenRep["right"] = "right"
  queryTokenRep["left"] = "left"
  queryTokenRep["next"] = "next"
  queryTokenRep["near"] = "nextLocation"
  queryTokenRep["front"] = "front"
  queryTokenRep["of"] = "of"

def getPredicate(role_filler):
  temp = role_filler
  rl = temp.replace("the", "")
  rl = rl.replace("my", "")
  rl = rl.replace("a", "")
  rl = rl.replace("some", "")
  rl = rl.replace("this", "")
  tokens = rl.split(" ")

  queryVector = []
  atomVector = []
  for token in tokens:
    if token in queryTokenRep:
      queryVector.append(queryTokenRep[token])

  ret = ""
  
  for i in range(0,len(queryVector)):
    if i > 0:
      queryVector[i] = queryVector[i].title()
    ret += (queryVector[i])
  
  #DEMO ONLY: for the frame "Arriving"
  if len(queryVector) == 0:
    ret += "location"
  
  return ret

def getArguments(role_filler):
  temp = role_filler
  rl = temp.replace("the", "")
  rl = rl.replace("my", "")
  rl = rl.replace("a", "")
  print rl
  tokens = rl.split(" ")
  queryVector = []
  atomVector = []
  for token in tokens:
    if token in queryTokenRep:
      queryVector.append(queryTokenRep[token])
    else:
     if token != "in" and token != "": 
       atomVector.append(token)
 
  atomVector.append("Y")
  atomVector.append("X")
  atomVector.append("Theta")
  return atomVector


def test():
    rospy.init_node('tester', anonymous=True)
    rospy.Subscriber("semantic_frames", frame, callback)
    rospy.spin()


if __name__ == '__main__':
    test()