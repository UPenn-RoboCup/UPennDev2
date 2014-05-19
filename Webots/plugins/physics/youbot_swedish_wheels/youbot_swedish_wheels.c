/*
 * File:          youbot_wheels.c
 * Date:          19th July 2011
 * Description:   Manage the simplified youbot wheels
 * Author:        fabien.rohrer@cyberbotics.com
 */

#include <plugins/physics.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#ifndef bool
#define bool char
#endif

#ifndef true
#define true ((bool) 1)
#endif

#ifndef false
#define false ((bool) 0)
#endif

#define MAX_CONTACTS 10

static pthread_mutex_t mutex;

static dGeomID ground_geom;

// YouBot structure and linked list
typedef struct YouBot {
  dBodyID wheels_bodies[4];
  dGeomID wheels_geoms[4];
  struct YouBot *next; // linked list
} YouBot;
static YouBot *youbot_list = NULL;

// print the expected usage of this plugin
static void usage() {
  dWebotsConsolePrintf("youbot_swedish_wheels plugin usage and constraints:");
  dWebotsConsolePrintf("  The ground object should be a Solid referenced by the 'GROUND' DEF-identifier, and should contain a Plane along the xz-axis.");
  dWebotsConsolePrintf("  In case of multiple youBots, the youBot robots should be referenced by the 'YOUBOTX' DEF-identifier where X is a digit between 0 and 9.");
  dWebotsConsolePrintf("  The maximum number of youBots is 10 (unless to modify this plugin).");
}

// create a youbot instance, and add it to the linked list
static void create_youbot(dBodyID wheels_bodies[4], dGeomID wheels_geoms[4]) {
  YouBot *youbot = (YouBot *) malloc( sizeof(YouBot) );
  
  int i;
  for (i = 0; i < 4; i++) {
    youbot->wheels_bodies[i] = wheels_bodies[i];
    youbot->wheels_geoms[i] = wheels_geoms[i];
  }

  youbot->next = youbot_list;
  youbot_list = youbot;
}

void webots_physics_init(dWorldID w, dSpaceID s, dJointGroupID j) {
  pthread_mutex_init(&mutex, NULL); // needed to run with multi-threaded version of ODE

  ground_geom = dWebotsGetGeomFromDEF("GROUND");
  if (ground_geom == NULL) {
    dWebotsConsolePrintf("youbot_swedish_wheels plugin: Cannot found the ground object.");
    usage();
  }

  int i, r;
  char name[32];
  dBodyID wheels_bodies[4];
  dGeomID wheels_geoms[4];

  // look for the YouBots refered from YOUBOT[0-9]
  for (r = 0; r < 10; r++) {

    sprintf(name, "YOUBOT%d", r);
    dBodyID robot_body = dWebotsGetBodyFromDEF(name);
    if (robot_body == NULL)
      continue; // useless to go further

    bool found = true;

    for (i = 0; i < 4; i++) {
      sprintf(name, "YOUBOT%d.WHEEL%d.WHEEL_SOLID", r, i + 1);
      wheels_bodies[i] = dWebotsGetBodyFromDEF(name);
      wheels_geoms[i] = dWebotsGetGeomFromDEF(name);
      if (wheels_bodies[i] == NULL || wheels_geoms[i] == NULL) {
        found = false; // useless to go further
        dWebotsConsolePrintf("youbot_swedish_wheels plugin: Cannot found the wheels of the robot 'YOUBOT%d'.", r);
        usage();
        break;
      }
    }

    if (found)
      create_youbot(wheels_bodies, wheels_geoms);
  }

  // if nothing is found, simply look for an undefined YouBot
  if (youbot_list == NULL) {
    bool found = true;
    for (i = 0; i < 4; i++) {
      sprintf(name, "WHEEL%d.WHEEL_SOLID", i + 1);
      wheels_bodies[i] = dWebotsGetBodyFromDEF(name);
      wheels_geoms[i] = dWebotsGetGeomFromDEF(name);
      if (wheels_bodies[i] == NULL || wheels_geoms[i] == NULL) {
        found = false; // useless to go further
        break;
      }
    }
    if (found)
      create_youbot(wheels_bodies, wheels_geoms);
  }
}

void webots_physics_step() {
}

int webots_physics_collide(dGeomID g1, dGeomID g2) {
  if (ground_geom == NULL || youbot_list == NULL)
    return 0; // initialisation error: collision must be handled by webots

  dBodyID body = dGeomGetBody(g1);
  if (body == NULL) body = dGeomGetBody(g2);
  if (body == NULL) return 0; // ignore collisions between geoms that have no body 
  const dWorldID world = dBodyGetWorld(body);

  // retrieve wheel_id in the youbot list
  int wheel_id = 0;
  bool wheel_id_found = false;

  YouBot *youbot = youbot_list;
  while (youbot) {
    for (wheel_id = 0; wheel_id < 4; wheel_id++) {
      if (dAreGeomsSame(g1, youbot->wheels_geoms[wheel_id]) || dAreGeomsSame(g2, youbot->wheels_geoms[wheel_id])) {
        wheel_id_found = true;
        break;
      }
    }
    if (wheel_id_found)
      break;
    youbot = youbot->next;
  }

  // deal with the contact
  if ((youbot && wheel_id_found && (dAreGeomsSame(g1, ground_geom) || dAreGeomsSame(g2, ground_geom))))
  {
    dContact contacts[MAX_CONTACTS];
    dJointID contact_joints[MAX_CONTACTS];
    dJointGroupID contact_joint_group = dWebotsGetContactJointGroup();

    // see how many collision points there are between theses objects
    int nContacts = dCollide(g1, g2, MAX_CONTACTS, &contacts[0].geom, sizeof(dContact));
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
        a = dBodyGetPosition(youbot->wheels_bodies[1]);
        b = dBodyGetPosition(youbot->wheels_bodies[2]);
        c = dBodyGetPosition(youbot->wheels_bodies[3]);
      }
      else // exterior wheel
      {
        a = dBodyGetPosition(youbot->wheels_bodies[0]);
        b = dBodyGetPosition(youbot->wheels_bodies[3]);
        c = dBodyGetPosition(youbot->wheels_bodies[2]);
      }
      dVector3 ac = {a[0] - c[0], a[1] - c[1], a[2] - c[2]};
      dVector3 bc = {b[0] - c[0], b[1] - c[1], b[2] - c[2]};
      dSafeNormalize3(ac);
      dSafeNormalize3(bc);
      contacts[i].fdir1[0] = (ac[0] + bc[0]);
      contacts[i].fdir1[1] = (ac[1] + bc[1]);
      contacts[i].fdir1[2] = (ac[2] + bc[2]);
      dSafeNormalize3(contacts[i].fdir1);
      
      // create a contact joint that will prevent the two bodies from intersecting
      // note that contact joints are added to the contact_joint_group
      // we need to lock a mutex when creating and attaching joints to allow safe multi-threaded ODE
      pthread_mutex_lock(&mutex);
      contact_joints[i] = dJointCreateContact(world, contact_joint_group, &contacts[i]);      
      // attach joint between the body and the static environment (0)
      dJointAttach(contact_joints[i], youbot->wheels_bodies[wheel_id], 0);
      pthread_mutex_unlock(&mutex);
    }
    return 2;  // collision was handled above and will be notified by an change in the boundingObject color
  }
  return 0;  // collision must be handled by webots
}

void webots_physics_cleanup() {
  pthread_mutex_destroy(&mutex);
}
