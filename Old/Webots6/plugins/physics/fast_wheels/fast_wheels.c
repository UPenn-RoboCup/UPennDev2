/*
 * File:          youbot_wheels.c
 * Date:          19th July 2011
 * Description:   Manage the simplified youbot wheels
 * Author:        fabien.rohrer@cyberbotics.com
 */

#include <ode/ode.h>
#include <plugins/physics.h>

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#define MAX_CONTACTS 10

static dWorldID world = NULL;
static dSpaceID space = NULL;
static dJointGroupID contact_joint_group = NULL;

static dBodyID wheels_bodies[4];
static dGeomID wheels_geoms[4];
static dBodyID ground_body;
static dGeomID ground_geom;

static int nContacts;
static dContact contacts[MAX_CONTACTS];
static dJointID contact_joints[MAX_CONTACTS];

void webots_physics_init(dWorldID w, dSpaceID s, dJointGroupID j) {
  world = w;
  space = s;
  contact_joint_group = j;

  char name[32];
  int i;
  
  for (i=0; i<4; i++) {
    sprintf(name, "WHEEL%d", i+1);
    wheels_bodies[i] = dWebotsGetBodyFromDEF(name);
    wheels_geoms[i] = dWebotsGetGeomFromDEF(name);
  }
  
  ground_body = dWebotsGetBodyFromDEF("GROUND");
  ground_geom = dWebotsGetGeomFromDEF("GROUND");
}

void webots_physics_step() {
}

void webots_physics_draw() {
}

int webots_physics_collide(dGeomID g1, dGeomID g2) {
  
  // retrieve wheel_id
  int wheel_id = -1;
  int wheel_id_found = 0;
  for (wheel_id=0; wheel_id<4; wheel_id++) {
    if (g1 == wheels_geoms[wheel_id] || g2 == wheels_geoms[wheel_id]) {
      wheel_id_found = 1;
      break;
    }
  }
  
  if ((wheel_id_found && (g1 == ground_geom || g2 == ground_geom)))
  {
    // see how many collision points there are between theses objects
    nContacts = dCollide(g1, g2, MAX_CONTACTS, &contacts[0].geom, sizeof(dContact));
    int i;
    const dReal *a, *b, *c;
    for (i = 0; i < nContacts; i++) {
      
      // custom parameters for creating the contact joint
      // remove or tune these contact parameters to suit your needs
      contacts[i].surface.mode = dContactApprox1 | dContactMu2 | dContactFDir1 | dContactSlip1;
      contacts[i].surface.mu = 2.0;
      contacts[i].surface.mu2 = 0.2;
      contacts[i].surface.slip1 = 10.0;
      
      // compute fdir1
      if (wheel_id == 0 || wheel_id == 3) // interior wheel
      {
        a = dBodyGetPosition(wheels_bodies[1]);
        b = dBodyGetPosition(wheels_bodies[2]);
        c = dBodyGetPosition(wheels_bodies[3]);
      }
      else // exterior wheel
      {
        a = dBodyGetPosition(wheels_bodies[0]);
        b = dBodyGetPosition(wheels_bodies[3]);
        c = dBodyGetPosition(wheels_bodies[2]);
      }
      dVector3 ac = {a[0]-c[0], a[1]-c[1], a[2]-c[2]};
      dVector3 bc = {b[0]-c[0], b[1]-c[1], b[2]-c[2]};
      dSafeNormalize3(ac);
      dSafeNormalize3(bc);
      contacts[i].fdir1[0] = (ac[0] + bc[0]);
      contacts[i].fdir1[1] = (ac[1] + bc[1]);
      contacts[i].fdir1[2] = (ac[2] + bc[2]);
      dSafeNormalize3(contacts[i].fdir1);
      
      // create a contact joint that will prevent the two bodies from intersecting
      // note that contact joints are added to the contact_joint_group
      contact_joints[i] = dJointCreateContact(world, contact_joint_group, &contacts[i]);
      
      // attach joint between the body and the static environment (0)
      dJointAttach(contact_joints[i], wheels_bodies[wheel_id], 0);
    }

    return 1;  // collision was handled above
  }

  return 0;  // collision must be handled by webots
}

void webots_physics_cleanup() {
}
