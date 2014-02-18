/*
(c) 2014 Stephen G. McGill
Kinematics for KUKA YouBot's 5 DOF arm
*/

#include "YouBotKinematics.h"

Transform YouBot_kinematics_forward_arm(const double *q) {
  Transform tr;
  tr.rotateZ(q[0])
    .translateX(baseLength) // do not use for now
    .rotateY(q[1])
    .translateZ(lowerArmLength)
    .rotateY(q[2])
    .translateZ(upperArmLength)
    .rotateY(q[3])
    .translateZ(wristLength+handLength)
    .rotateZ(q[4]);
  return tr;
}

std::vector<double> YouBot_kinematics_com_arm(const double *q, std::vector<double>& comObject, const double mObject) {
  Transform tr_lower_arm, tr_upper_arm, tr_wrist, tr_hand, tr_object;
	std::vector<double> com(3);
	
	tr_lower_arm
    .rotateZ(q[0])
    .translateX(baseLength)
    .rotateY(q[1]);
	// base ends pointing in the direction that the lower arm goes
  tr_upper_arm = trcopy(tr_lower_arm)
		.translateZ(lowerArmLength)
		.rotateY(q[2]);
	// upper arm ends pointing in the direction that the upper arm goes
	tr_wrist = trcopy(tr_upper_arm)
    .translateZ(upperArmLength)
		.rotateY(q[3]);
	// lower arm ends pointing in the direction that the wrist goes
	tr_hand = trcopy(tr_wrist)
    .translateZ(wristLength)
		.rotateZ(q[4]);
	// wrist ends twisting in the direction that the hand is twisted
	tr_object = trcopy(tr_hand)
		.translateZ(handLength);
	// hand ends pointing in the direction of the object it holds
	
	// Now add the offsets for the masses
	tr_lower_arm
		.translateX(comLowerArm[0])
		.translateX(comLowerArm[1])
    .translateZ(comLowerArm[2]);
	tr_upper_arm
		.translateX(comUpperArm[0])
		.translateX(comUpperArm[1])
    .translateZ(comUpperArm[2]);
	tr_wrist
		.translateX(comWrist[0])
		.translateX(comWrist[1])
    .translateZ(comWrist[2]);
	tr_hand
		.translateX(comHand[0])
		.translateX(comHand[1])
    .translateZ(comHand[2]);
	tr_object
		.translateX(comObject[0])
		.translateX(comObject[1])
    .translateZ(comObject[2]);
	
	// Form the centor of mass vector
	com[0] = mLowerArm * tr_lower_arm(0,3)
		+ mUpperArm * tr_upper_arm(0,3)
		+ mWrist * tr_wrist(0,3)
		+ mHand * tr_hand(0,3)
		+ mObject * tr_object(0,3);
	
	com[1] = mLowerArm * tr_lower_arm(1,3)
		+ mUpperArm * tr_upper_arm(1,3)
		+ mWrist * tr_wrist(1,3)
		+ mHand * tr_hand(1,3)
		+ mObject * tr_object(1,3);
	
	com[2] = mLowerArm * tr_lower_arm(2,3)
		+ mUpperArm * tr_upper_arm(2,3)
		+ mWrist * tr_wrist(2,3)
		+ mHand * tr_hand(2,3)
		+ mObject * tr_object(2,3);
	
  return com;
}


// Given the appropriate yaw, just get the angles in the xz plane
// Input: x and z position, pitch angle
// Output: Vector of 3 pitch angles
std::vector<double> get_xz_angles(double xy_dist, double z, double p ){

	double xy_coord = xy_dist - baseLength;
  // Given the "pitch", find the effective position
  double dx = xy_coord - gripperLength * sin(p);
  double dz = z - gripperLength * cos(p);
#ifdef DEBUG
/*
  printf("\tdx: %lf, dz: %lf\n",dx,dz);
*/
#endif
  
  // Find the elbow, given the effective position
  // Law of cosines
	double dr2 = dx*dx+dz*dz;
  double dr  = sqrt(dr2);
  double elbow_help = upperArmLength*upperArmLength + lowerArmLength*lowerArmLength - dr2;
  elbow_help /= 2*upperArmLength*lowerArmLength;
#ifdef DEBUG
/*
  printf("\telbow_help: %lf\n",elbow_help);
*/
#endif
  if(elbow_help<-1)
    elbow_help = -1;
  else if(elbow_help>1)
    elbow_help = 1;
  
  double elbow = acos(elbow_help);
  // Find how much to rotate the shoulder pitch
  // Law of sines
  double ratio = upperArmLength * sin(elbow) / dr;
  double effective_elevation = asin(ratio);
  double elevation = PI/2 - atan2(dz,dx);
  double shoulderPitch = elevation - effective_elevation;

#ifdef DEBUG
/*
  printf("\telbow: %lf\n",elbow);
	printf("\tratio: %lf\n",ratio);
	printf("\teffective_elevation: %lf\n",effective_elevation);
	printf("\televation: %lf\n",elevation);
*/
#endif

  // Form the vector
  std::vector<double> xz(5);
  xz[0] = 0;
	xz[1] = shoulderPitch;
  xz[2] = PI - elbow;
  xz[3] = p - (shoulderPitch + (PI - elbow));
	if(xz[3]>PI)
		xz[3] -= PI_DOUBLE;
	else if(xz[3]<-PI)
		xz[3] += PI_DOUBLE;
	xz[4] = 0;

  return xz;
}

// Inverse given a transform
// Default uses the safe yaw
std::vector<double> YouBot_kinematics_inverse_arm(Transform tr, std::vector<double>& q, char& is_reach_back, bool use_safe_yaw=true) {
  double dx, dy, dz, base_yaw, pseudo_yaw, pitch, hand_yaw, tmp1, xy_coord;
	char unique_pitch, yaw_issue;

	// Grab the pitch
	tmp1 = tr( 2, 2 );
	unique_pitch = tmp1 > .999 ? 1 : (tmp1 < -.999 ? -1 : 0);
	// Check for singularity when the z1 and z2 yaws are parallel
	// Form the yaw of the hand
	if( unique_pitch!=0 ){
		pitch = unique_pitch > 0 ? 0 : PI;
		pseudo_yaw = 0;
		hand_yaw = asin( tr( 1, 0 ) );
#ifdef DEBUG
		printf("\nUndefined pitch %d\n",unique_pitch);
#endif
	} else {
		pitch = acos( tmp1 );
		pseudo_yaw = atan2( tr(1,2),tr(0,2) );
		hand_yaw = atan2( tr(2,1),-tr(2,0) );
#ifdef DEBUG
		printf("\nWell defined pitch\n");
#endif
	}

	// Grab the position
  dx = tr(0,3);
  dy = tr(1,3);
  dz = tr(2,3);
  xy_coord = sqrt(dx*dx + dy*dy);
	// Check vertical singularity
	if( xy_coord<1e-9 ) {
		base_yaw = q[0];
	} else {
		base_yaw = atan2(dy,dx);
		// Decide to reach back
		if( (base_yaw-q[0])>=PI_ALMOST ){
#ifdef DEBUG
			printf("\tReach back +\n");
#endif
			base_yaw -= PI;
			xy_coord *= -1;
			is_reach_back = 1;
		} else if( (q[0]-base_yaw)>=PI_ALMOST ){
#ifdef DEBUG
			printf("\tReach back -\n");
#endif
			base_yaw += PI;
			xy_coord *= -1;
			is_reach_back = -1;
		} else
			is_reach_back = 0;
	} // End check if close to vertical singularity
	
#ifdef DEBUG
	printf("\tbase_yaw: %lf\n",base_yaw);
	printf("\tq[0]: %lf\n",q[0]);
	printf("\treach_back: %d\n",is_reach_back);
	printf("\tpitch: %lf\n",pitch);
	printf("\tp_yaw: %lf\n",pseudo_yaw);
	printf("\th_yaw: %lf\n",hand_yaw);
#endif
	
	yaw_issue = 0;
	if( (base_yaw-pseudo_yaw)>=PI_HALF ){
#ifdef DEBUG
		printf("\tYaw issue +\n");
#endif
		pseudo_yaw += PI;
		hand_yaw -= PI;
		pitch *= -1;
		yaw_issue = 1;
	} else if( (pseudo_yaw-base_yaw)>=PI_HALF ){
#ifdef DEBUG
		printf("\tYaw issue -\n");
#endif
		pseudo_yaw -= PI;
		hand_yaw -= PI;
		pitch *= -1;
		yaw_issue = -1;
	}

#ifdef DEBUG
	printf("\tpitch: %lf\n",pitch);
	printf("\tbase_yaw: %lf\n",base_yaw);
	printf("\tpseudo_yaw: %lf\n",pseudo_yaw);
	printf("\thand_yaw: %lf\n",hand_yaw);
#endif

	// Make into positive space for the pitch (above PI)
	if( pitch<0 && is_reach_back==0 ){
		pitch += PI_DOUBLE;
		// Also flip the hand_yaw...?
	}

	if(hand_yaw>PI)
		hand_yaw -= PI_DOUBLE;
	else if(hand_yaw<-PI)
		hand_yaw += PI_DOUBLE;

	// Optional safe yaw:
	if(use_safe_yaw){
		double diff_yaw = pseudo_yaw - base_yaw;
		hand_yaw += cos(pitch) * diff_yaw;
	}

#ifdef DEBUG
	printf("\tdiff_yaw: %lf\n",diff_yaw);
	printf("\thand_yaw: %lf\n",hand_yaw);
	printf("\tmod_yaw: %lf\n",hand_yaw);
	printf("\tpitch, cos(pitch): %lf %lf\n",pitch,cos(pitch));
#endif

	// Grab the XZ plane angles
  std::vector<double> output = get_xz_angles( xy_coord, dz, pitch );
  output[0] = base_yaw;
	output[4] = hand_yaw;
	return output;
  
}

// Inverse given only a position
std::vector<double> YouBot_kinematics_inverse_arm_position(std::vector<double>& position, std::vector<double>& q, char& is_reach_back) {
	double dx = position[0], dy = position[1], dz = position[2];
	double xy_coord = sqrt(dx*dx + dy*dy);
	// Check if we want to reach back
	double base_yaw;
	if( xy_coord<1e-6 )
		base_yaw = q[0];
	else {
		base_yaw = atan2(dy,dx);
		// Decide to reach back
		if( (base_yaw-q[0])>=PI_ALMOST ){
			base_yaw -= PI;
			xy_coord *= -1;
			is_reach_back = 1;
		} else if( (q[0]-base_yaw)>=PI_ALMOST ){
			base_yaw += PI;
			xy_coord *= -1;
			is_reach_back = -1;
		} else is_reach_back = 0;
	}
	double xy_dist = xy_coord - baseLength;
	double extended_arm_length = sqrt( xy_dist*xy_dist + dz*dz );
	double arm_ratio = extended_arm_length / armLength;
	arm_ratio = arm_ratio>1?1:(arm_ratio<0?0:arm_ratio);

	// Start guessing the pitch
	double pitch0 = atan2(xy_dist,dz);
	double pitch1 = pitch0 + PI_HALF;
	double pitch = pitch0 * arm_ratio + pitch1 * (1-arm_ratio);

#ifdef DEBUG
	printf("\arm_ratio: %lf\n",arm_ratio);
	printf("\pitch0: %lf\n",pitch0);
	printf("\pitch1: %lf\n",pitch1);
#endif

// Grab the XZ plane angles
  std::vector<double> output = get_xz_angles( xy_coord, dz, pitch );
  output[0] = base_yaw;
	// Keep the same angle of the gripper yaw
	output[4] = q[4];
	return output;

}
