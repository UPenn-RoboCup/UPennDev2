/*
(c) 2014 Stephen G. McGill
Kinematics for KUKA YouBot's 5 DOF arm
*/

#include "YouBotKinematics.h"

void printTransform(Transform tr) {
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      printf("%.4g ", tr(i,j));
    }
    printf("\n");
  }
  printf("\n");
}

void printVector(std::vector<double> v) {
  for (int i = 0; i < v.size(); i++) {
    printf("%.4g\n", v[i]);
  }
  printf("\n");
}

Transform YouBot_kinematics_forward_arm(const double *q) {
  Transform t;
  t = t
    .rotateZ(q[0])
    .translateX(baseLength) // do not use for now
    .rotateY(q[1])
    .translateZ(lowerArmLength)
    .rotateY(q[2])
    .translateZ(upperArmLength)
    .rotateY(q[3])
    .translateZ(wristLength+handLength)
    .rotateZ(q[4]);
  return t;
}

// Given the appropriate yaw, just get the angles in the xz plane
// Input: x and z position, pitch angle
// Output: Vector of 3 pitch angles
std::vector<double> get_xz_angles(double xy_dist, double z, double p ){

	printf("==\n\txy_dist: %lf\n",xy_dist);
	printf("\n\tz: %lf\n",z);
	printf("\n\tp: %lf\n",p);

	double xy_coord = xy_dist - baseLength;
  // Given the "pitch", find the effective position
  double dx = xy_coord - gripperLength * sin(p);
  double dz = z - gripperLength * cos(p);
	
  printf("\tdx: %lf, dz: %lf\n",dx,dz);
  
  double dr2 = dx*dx+dz*dz;
  double dr  = sqrt(dr2);
  
  //printf("\n\televation: %lf\n",elevation);
  
  // Find the elbow, given the effective position
  // Law of cosines
  double elbow_help = upperArmLength*upperArmLength + lowerArmLength*lowerArmLength - dr2;
  elbow_help /= 2*upperArmLength*lowerArmLength;
  //printf("\n\telbow_help: %lf\n",elbow_help);
  if(elbow_help<-1)
    elbow_help = -1;
  else if(elbow_help>1)
    elbow_help = 1;
  
  double elbow = acos(elbow_help);
  //printf("\n\telbow: %lf\n",elbow);
  
  // Find how much to rotate the shoulder pitch
  // Law of sines
  double ratio = upperArmLength * sin(elbow) / dr;
  
  //printf("\n\tRatio: %lf\n",ratio);
  
  double effective_elevation = asin(ratio);
  
  //printf("\n\teffective_elevation: %lf\n",effective_elevation);
  double elevation = PI/2 - atan2(dz,dx);
  double shoulderPitch = elevation - effective_elevation;

  // Form the vector
  std::vector<double> xz(5);
  xz[0] = 0;
	xz[1] = shoulderPitch;
  xz[2] = PI - elbow;
  xz[3] = p - (shoulderPitch + (PI - elbow));
	xz[4] = 0;
	//printf("\tp: %lf\n\tSum: %lf\n\tdiff: %lf\n",p,shoulderPitch + xz[1], xz[2]);

  return xz;
}

// Inverse given a transform
std::vector<double> YouBot_kinematics_inverse_arm(Transform tr, std::vector<double> q) {
  double dx, dy, dz, base_yaw, pseudo_yaw, pitch, hand_yaw, tmp1, xy_coord;
	//double yaw, dyaw, tmp2, ;
	char unique_pitch, reach_back, yaw_issue;

	// Grab the pitch
	tmp1 = tr( 2, 2 );
	unique_pitch = tmp1 > .999 ? 1 : (tmp1 < -.999 ? -1 : 0);
	// Check for singularity when the z1 and z2 yaws are parallel
	// Form the yaw of the hand
	if( unique_pitch!=0 ){
		pitch = unique_pitch > 0 ? 0 : PI;
		pseudo_yaw = 0;
		hand_yaw = asin( tr( 1, 0 ) );
		printf("\nUndefined pitch\n");
	} else {
		pitch = acos( tmp1 );
		pseudo_yaw = atan2( tr(1,2),tr(0,2) );
		hand_yaw = atan2( tr(2,1),-tr(2,0) );
		printf("\nWell defined pitch\n");
	}

	// Grab the position
  dx = tr(0,3);
  dy = tr(1,3);
  dz = tr(2,3);
  xy_coord = sqrt(dx*dx + dy*dy);
	// Check if we want to reach back
	if( xy_coord<1e-9 ) {
		base_yaw = q[0];
		if( unique_pitch==0 ){printf("\tSPECIAL SCENARIO 1\n");}
	} else {
		base_yaw = atan2(dy,dx);
	}

	printf("\tbase_yaw: %lf\n",base_yaw);
	printf("\tq[0]: %lf\n",q[0]);

	reach_back = 0;
	if( (base_yaw-q[0])>=PI_HALF ){
		// Decide to reach back
		printf("\tCheck if we wish to reach back +\n");
		base_yaw -= PI;
		xy_coord *= -1;
		reach_back = 1;
	} else if( (q[0]-base_yaw)>=PI_HALF ){
		// Decide to reach back
		printf("\tCheck if we wish to reach back -\n");
		base_yaw += PI;
		xy_coord *= -1;
		reach_back = -1;
	}
	
	printf("\treach_back: %d\n",reach_back);
	printf("\tpitch: %lf\n",pitch);
	printf("\tp_yaw: %lf\n",pseudo_yaw);
	printf("\th_yaw: %lf\n",hand_yaw);
	
	yaw_issue = 0;
	if( (base_yaw-pseudo_yaw)>=PI_HALF ){
		printf("\tYaw issue +\n");
		pseudo_yaw += PI;
		hand_yaw += PI;
		pitch *= -1;
		yaw_issue = 1;
	} else if( (pseudo_yaw-base_yaw)>=PI_HALF ){
		printf("\tYaw issue -\n");
		pseudo_yaw -= PI;
		hand_yaw -= PI;
		pitch *= -1;
		yaw_issue = -1;
	}

	printf("\tpitch: %lf\n",pitch);
	printf("\tbase_yaw: %lf\n",base_yaw);
	printf("\tpseudo_yaw: %lf\n",pseudo_yaw);
	printf("\thand_yaw: %lf\n",hand_yaw);

	// Make into positive space for the pitch (above PI)
	if( pitch<0 && reach_back==0 ){
		pitch += PI_DOUBLE;
	}

	// Grab the XZ plane angles
  std::vector<double> output = get_xz_angles( xy_coord, dz, pitch );
  output[0] = base_yaw;
  //output[4] = hand_yaw;
	return output;



	/*
	if( tmp1 > .99 || tmp1 < -.99 ){
		p = tmp1 > 0 ? 0 : PI;
		printf("\nUndefined pitch\n");
		z1 = asin( tr( 1, 0 ) );
		// Check if around zero
		if( x<1e-9 && x>-1e-9 && y<1e-9 && y>-1e-9 ) {
			// Use the current base
			yaw = q[0]; // Current Yaw
			hand_yaw = z1;
			//printf("Undefined pitch around zero yaw:\n\t%lf\n\t%lf\n",yaw,z1);
		} else {
			// Determine the angle
			yaw = atan2(y,x);
			hand_yaw = z1 + yaw;
			printf("\tyaw: %lf, z1: %lf\n",yaw,z1);
			printf("\thand_yaw: %lf\n",hand_yaw);
		}
	} else {
		p = acos( tmp1 );
		printf("\nWell defined pitch\n");

		// Grab also the "yaw" of the gripper (ZYZ, where 2nd Z is yaw)
		tmp1 =  tr( 2, 1 );
		tmp2 = -tr( 2, 0 );
		hand_yaw = atan2(tmp1,tmp2);
		
		// Grab also the "yaw" of the base (ZYZ, where 1st Z is yaw)
		tmp1 = tr( 0, 2 );
		tmp2 = tr( 1, 2 );
		z1 = atan2(tmp2,tmp1);

		// Check if position is around zero
		if( x<1e-9 && x>-1e-9 && y<1e-9 && y>-1e-9 ) {
			// Use the current base
			yaw = q[0];
			dyaw = z1 > yaw ? z1 - yaw : yaw - z1;
			//printf("Around zero yaw:\n\t%lf\n\t%lf\n",yaw,z1);
		} else {
			// Determine the angle
			yaw = atan2(y,x);
			//printf("OK pitch | z1,yaw,p:\n\t%lf\n\t%lf\n\t%lf\n\t%lf\n\t%lf\n",z1,yaw,p,yaw*cos(p),hand_yaw);
			// This may be good...
			//hand_yaw -= yaw*cos(p);
		}
	}
	
	// Check against our current base angle
	dyaw = q[0] > yaw ? q[0] - yaw : yaw - q[0];
	if(dyaw>=PI_HALF){
		// Major difference in current and proposed base yaw
		xy_coord = -xy_dist - baseLength;
		yaw = q[0];
		prtinf("Difference\n");
	}
	*/

  // Grab the XZ plane angles

  
}

// Inverse given only a position
std::vector<double> YouBot_kinematics_inverse_arm_position(double x, double y, double z) {

  // Find the perfect pitch

  // Remove rotation of the shoulder yaw
  double dist2 = x*x + y*y;
  double dist = sqrt(dist2);
  double yaw = atan2(y,x);
  // x is the distance now, less the offset
  x = dist - baseLength;

  // TODO: Make a simple optimization routine
  // How much the arm must reach
  double dr2 = x*x+z*z;
  double dr  = sqrt(dr2);
  double p;
  if(dr<.35) {
    // Optimization routine in tight spaces
    // Basically loop between 90 and 180 degrees
    // TODO: Could in clude the *current* pitch angle
    double ratio = (dr-.1)/.3;
    if(ratio<0) ratio=0;
    p = ratio * PI/2 + (1-ratio)*PI*125/180;
    printf("pp: %lf, ratio: %lf\n",p,ratio);
  } else {
    // The angle of attack is our perfect pitch
    p = atan2(x,z);
  }
  // End perfect pitch calculation

  // Grab the XZ plane angles
  std::vector<double> xz = get_xz_angles( x, z, p );

  // Output to joint angles
  std::vector<double> qArm(5);
  qArm[0] = yaw;
  qArm[1] = xz[0];
  qArm[2] = xz[1];
  qArm[3] = xz[2];
  qArm[4] = 0; // Go to zero here, but this is arbitrary

  return qArm;
}
