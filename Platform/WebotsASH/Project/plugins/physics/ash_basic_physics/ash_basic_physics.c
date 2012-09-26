#include <ode/ode.h>
#include <GL/gl.h>
#include <plugins/physics.h>
#include <string.h>

// ash_basic_physics.c 
// refer to stewart_platform_physics.c and contact_points_physics.c for reference

#define N_SERVO               12
#define FTS_RECEIVER_CHANNEL  10
#define MAX_CONTACTS          10
#define MAX_COP               10
#define DRAW_COP              1
#define DRAW_CONTACT_POINTS   0

// define global variables 
static dWorldID world = NULL;
static dSpaceID space = NULL;
static dJointGroupID contact_joint_group = NULL;

// define foot / floor contact variables
enum foot {LEFT_FOOT, RIGHT_FOOT, N_FEET};
static dGeomID floor_geom = NULL;
static dGeomID foot_geom[2] = {NULL, NULL};
static dBodyID foot_body[2] = {NULL, NULL};
static int nContacts[2] = {0, 0};
static dContact foot_contacts[2][MAX_CONTACTS];
static dJointFeedback foot_feedbacks[2][MAX_CONTACTS];
static double foot_fts[2][6] = {
  {0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0},
};
static double foot_cop[MAX_COP][2];

// define foot / floor names
static const char *floor_name = "FLOOR";
static const char *foot_name[2] = {
  "L_ANKLE_P2",
  "R_ANKLE_P2",
};

// define webots servo names
static const char *servo_name[N_SERVO] = {
  "L_HIPYAW", 
  "L_HIPROLL",
  "L_HIPPITCH", 
  "L_KNEE_PIVOT",
  "L_ANKLE",
  "L_ANKLE_P2",
  "R_HIPYAW",
  "R_HIPROLL",
  "R_HIPPITCH", 
  "R_KNEE_PIVOT",
  "R_ANKLE",
  "R_ANKLE_P2",
};

// convenience function to print a 3d vector
void print_vec3(const char *msg, const dVector3 v) {
  dWebotsConsolePrintf("%s: %g %g %g\n", msg, v[0], v[1], v[2]);
}

// convenience function to find ODE geometry by its DEF name in the .wbt file
dGeomID getGeom(const char *def) {
  dGeomID geom = dWebotsGetGeomFromDEF(def);
  if (!geom)
    dWebotsConsolePrintf("Warning: did not find geometry with DEF name: %s\n", def);
  return geom;
}

// convenience function to find ODE body by its DEF name in the .wbt file
dBodyID getBody(const char *def) {
  dBodyID body = dWebotsGetBodyFromDEF(def);
  if (!body)
    dWebotsConsolePrintf("Warning: did not find body with DEF name: %s\n", def);
  return body;
}

// debug function for rendering center of pressure and foot contact points 
void draw_cop() {

  int i, j;
  double total_pressure = 0;
  double cop_points[MAX_COP-1][2];
  for (i = MAX_COP; i > 0; i--)
    memcpy(foot_cop[i], foot_cop[i-1], 2*sizeof(double));
  memset(foot_cop[0], 0, 2*sizeof(double));

  // change OpenGL state
  glDisable(GL_LIGHTING);    // not necessary
  glLineWidth(4);            // use a thick line
  glDisable(GL_DEPTH_TEST);  // draw in front of Webots graphics

  for (i = 0; i < N_FEET; i++) {
    for (j = 0; j < nContacts[i]; j++) {
      dReal *p = foot_contacts[i][j].geom.pos;
      dReal *n = foot_contacts[i][j].geom.normal;
      dReal d = foot_contacts[i][j].geom.depth * 15;
      foot_cop[0][0] += p[0] * n[2] * d;
      foot_cop[0][1] += p[1] * n[2] * d;
      total_pressure += n[2] * d;

      // draw contact points
      if (DRAW_CONTACT_POINTS) {
        glBegin(GL_LINES);
        glColor3f(0, 1, 0);
        glVertex3f(p[0], p[1], p[2]);
        glVertex3f(p[0] + n[0] * d, p[1] + n[1] * d, p[2] + n[2] * d);
        glEnd();
      }
    }
  }
  foot_cop[0][0] /= total_pressure;
  foot_cop[0][1] /= total_pressure;

  // draw center of pressure trajectory
  for (i = 0; i < MAX_COP-1; i++) {
    cop_points[i][0] = (foot_cop[i][0] + foot_cop[i+1][0])/2;
    cop_points[i][1] = (foot_cop[i][1] + foot_cop[i+1][1])/2;
  }
  glBegin(GL_LINE_STRIP);
  glColor3f(1, 1, 1);
  for (i = 0; i < MAX_COP-1; i++) 
    glVertex3f(cop_points[i][0], cop_points[i][1], 0);
  glEnd( );
}

// called by Webots at the beginning of the simulation
void webots_physics_init(dWorldID w, dSpaceID s, dJointGroupID j) {

  int i;

  // store global objects for later use
  world = w;
  space = s;
  contact_joint_group = j;

  // get floor geometry
  floor_geom = getGeom(floor_name);
  if (!floor_geom)
    return;

  // get foot geometry and body id's
  for (i = 0; i < N_FEET; i++) {
    foot_geom[i] = getGeom(foot_name[i]);
    if (!foot_geom[i])
      return;
    foot_body[i] = dGeomGetBody(foot_geom[i]);
    if (!foot_body[i])
      return;
  }
}

// implemented to overide Webots collision detection,
// returns 1 if a specific collision is handled, and 0 otherwise
int webots_physics_collide(dGeomID g1, dGeomID g2) {

  static dJointID contact_joints[MAX_CONTACTS];
  
  int i, j;
  for (i = 0; i < N_FEET; i++) {
    // handle any collisions involving a foot and the floor
    if ((g1 == foot_geom[i] && g2 == floor_geom) || (g2 == foot_geom[i] && g1 == floor_geom)) {

      // see how many collision points there are between the two geometries
      nContacts[i] = dCollide(foot_geom[i], floor_geom, MAX_CONTACTS, &foot_contacts[i][0].geom, sizeof(dContact));

      for (j = 0; j < nContacts[i]; j++) {
        // custom parameters for creating the contact joint
        // remove or tune these contact parameters to suit your needs
        foot_contacts[i][j].surface.mode = dContactBounce | dContactSoftCFM | dContactApprox1;
        foot_contacts[i][j].surface.mu = 1.5;
        foot_contacts[i][j].surface.bounce = 0.5;
        foot_contacts[i][j].surface.bounce_vel = 0.01;
        foot_contacts[i][j].surface.soft_cfm = 0.001;
    
        // create a contact joint that will prevent the two bodies from intersecting
        // note that contact joints are added to the contact_joint_group
        contact_joints[j] = dJointCreateContact(world, contact_joint_group, &foot_contacts[i][j]);
    
        // attach joint between the body and the static environment (0)
        dJointAttach(contact_joints[j], foot_body[i], 0);
    
        // attach feedback structure to measure the force on the contact joint
        dJointSetFeedback(contact_joints[j], &foot_feedbacks[i][j]);
      }
      return 1;  // collision was handled above
    }
  }

  return 0;  // collision must be handled by webots
}

// called by Webots for every WorldInfo.basicTimeStep
void webots_physics_step() {

  // get feedforward torques from emitter channel 0
  int i, size = 0;
  double servo_torque[N_SERVO];
  void *data = dWebotsReceive(&size);
  if (size == sizeof(double)*N_SERVO) {
    memcpy(servo_torque, data, size);
  }

  // add feedforward torque to each servo
  for (i = 0; i < N_SERVO; i++) {
    // find dJointID for the hinge joint at each servo
    dBodyID body_id = getBody(servo_name[i]);
    int njoints = dBodyGetNumJoints(body_id);
    dJointID joint_id = dBodyGetJoint(body_id, njoints - 1);
    // add torque directly to the joint
    dJointAddHingeTorque(joint_id, -servo_torque[i]);
  }
}

// called by Webots after dWorldStep()
void webots_physics_step_end() {

  int i,j;

  // calculate force-torque measurements for foot contacts
  for (i = 0; i < N_FEET; i++) {

    double past_fts[6];
    double filtered_fts[6];
    memcpy(past_fts, foot_fts[i], 6*sizeof(double));
    memset(foot_fts[i], 0, 6*sizeof(double));

    // sum contact forces and torques
    for (j = 0; j < nContacts[i]; j++) {
      double *f = foot_feedbacks[i][j].f1;
      double *t = foot_feedbacks[i][j].t1;
      foot_fts[i][0] += f[0];
      foot_fts[i][1] += f[1];
      foot_fts[i][2] += f[2];
      foot_fts[i][3] += t[0];
      foot_fts[i][4] += t[1];
      foot_fts[i][5] += t[2];
    }

    // average current and past measurements
    for (j = 0; j < 6; j++) {
      filtered_fts[j] = (foot_fts[i][j] + past_fts[j])/2;
    }

    // send force-torque array to Webots receiver sensor
    dWebotsSend(FTS_RECEIVER_CHANNEL + i, filtered_fts, 6*sizeof(double)); 
  }
}

// called by Webots at each graphics step
void webots_physics_draw() {

  if (DRAW_COP)
    draw_cop();

}

// called by Webots to cleanup resources
void webots_physics_cleanup() {
  // nothing to cleanup ...
}

