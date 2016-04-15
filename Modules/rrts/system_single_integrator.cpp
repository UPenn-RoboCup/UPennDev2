#include "system_single_integrator.h"
#include <cmath>
#include <cstdlib>

//#include <iostream>

using namespace std;
using namespace SingleIntegrator;

//#define DISCRETIZATION_STEP 0.01
#define DISCRETIZATION_STEP 0.03
#define JANGLES
#define FK

extern double* xyzG;
extern Transform trG;

region::region () {

	numDimensions = 0;

	center = NULL;
	size = NULL;
}


region::~region () {

	if (center){
		delete [] center;
	} 
	if (size){
		delete [] size;
	}

}


int region::setNumDimensions (int numDimensionsIn) {

	numDimensions = numDimensionsIn;

	if (center){
		delete [] center;
	}
	center = new double[numDimensions];

	if (size){
		delete [] size;
	}
	size = new double[numDimensions];

	return 1;
}

int region::getNumDimensions () {

	return numDimensions;
}


State::State () {

	numDimensions = 0;

	x = NULL;
}


State::~State () {

	if (x)
		delete [] x;
}


State::State (const State &stateIn) {

	numDimensions = stateIn.numDimensions;

	if (numDimensions > 0) {
		x = new double[numDimensions];

		for (int i = 0; i < numDimensions; i++) 
			x[i] = stateIn.x[i];
	}
	else {
		x = NULL;
	}
}


State& State::operator=(const State &stateIn){

	if (this == &stateIn)
		return *this;

	if (numDimensions != stateIn.numDimensions) {
		if (x) 
			delete [] x;
		numDimensions = stateIn.numDimensions;
		if (numDimensions > 0)
			x = new double[numDimensions];
	}

	for (int i = 0; i < numDimensions; i++) 
		x[i] = stateIn.x[i];

	return *this;
}


int State::setNumDimensions (int numDimensionsIn) {

	if (x)
		delete [] x;

	if (numDimensions < 0)
		return 0;

	numDimensions = numDimensionsIn;

	if (numDimensions > 0)
		x = new double[numDimensions];

	return 1;
}

int State::getNumDimensions () {
	return numDimensions;
}


Trajectory::Trajectory () {

	endState = NULL;
}


Trajectory::~Trajectory () {

	if (endState)
		delete endState;
}


Trajectory::Trajectory (const Trajectory &trajectoryIn) {

	endState = new State (trajectoryIn.getEndState()); 

}


Trajectory& Trajectory::operator=(const Trajectory &trajectoryIn) {

	if (this == &trajectoryIn)
		return *this;

	if (endState)
		delete endState;


	endState = new State (trajectoryIn.getEndState());

	totalVariation = trajectoryIn.totalVariation;

	return *this;
}


double Trajectory::evaluateCost () {

	return totalVariation;
}


System::System () {

	numDimensions = 0;
}


System::~System () {

}


int System::setNumDimensions (int numDimensionsIn) {

	if (numDimensions < 0)
		return 0;

	numDimensions = numDimensionsIn;

	rootState.setNumDimensions (numDimensions);

	return 1;
}


int System::getStateKey (State& stateIn, double* stateKey) {

	for (int i = 0; i < numDimensions; i++) 
		stateKey[i] =  stateIn.x[i] / regionOperating.size[i];

	return 1;
}

int System::getTrajectory (State& stateFromIn, State& stateToIn, list<double*>& trajectoryOut) {

	double *stateArr = new double[numDimensions];
	for (int i = 0; i < numDimensions; i++)
		stateArr[i] = stateToIn[i];
	trajectoryOut.push_front (stateArr);

	return 1;

}

Transform System::fkLeft7(const double *q, const double *qWaist, 
		double handOffsetXNew, double handOffsetYNew, double handOffsetZNew){
	//FK for 7-dof arm (pitch-roll-yaw-pitch-yaw-roll-yaw)
	Transform t;

	t = t
    /*
		.translateZ(-originOffsetZ)
		.rotateY(qWaist[1]).rotateZ(qWaist[0])    
		.translateZ(originOffsetZ)
    */
		.translateY(shoulderOffsetY)
		.translateZ(shoulderOffsetZ)
		.mDH(-M_PI/2, 0, q[0], 0)
		.mDH(M_PI/2, 0, M_PI/2+q[1], 0)
		.mDH(M_PI/2, 0, M_PI/2+q[2], upperArmLengthL)
		.mDH(M_PI/2, elbowOffsetX, q[3], 0)
		.mDH(-M_PI/2, -elbowOffsetX, -M_PI/2+q[4], lowerArmLengthL)
		.mDH(-M_PI/2, 0, q[5], 0)
		.mDH(M_PI/2, 0, q[6], 0)
		.mDH(-M_PI/2, 0, -M_PI/2, 0)
		.translateX(handOffsetXNew)
		.translateY(-handOffsetYNew)
		.translateZ(handOffsetZNew);    

	return t;
}

Transform System::fkLeftElbow7(const double *q, const double *qWaist){
	//FK for 7-dof arm (pitch-roll-yaw-pitch-yaw-roll-yaw)
	Transform t;

	t = t
    /*
		.translateZ(-originOffsetZ)
		.rotateY(qWaist[1]).rotateZ(qWaist[0])    
		.translateZ(originOffsetZ)
    */
		.translateY(shoulderOffsetY)
		.translateZ(shoulderOffsetZ)
		.mDH(-M_PI/2, 0, q[0], 0)
		.mDH(M_PI/2, 0, M_PI/2+q[1], 0)
		.mDH(M_PI/2, 0, M_PI/2+q[2], upperArmLengthL)
		.mDH(M_PI/2, elbowOffsetX, q[3], 0);

	return t;
}

int System::sampleState (State &randomStateOut) {

	randomStateOut.setNumDimensions (numDimensions);

	// Bias 10% of the time
  double bias = (double)rand() / RAND_MAX;
	if(regionGoal.getNumDimensions()>0 && bias < 0.001){
		printf("Biasing... %f\n", bias);
		for (int i = 0; i < numDimensions; i++) {
			randomStateOut.x[i] = max(
				-regionOperating.size[i]/2.0,
				min(regionOperating.size[i]/2.0,
					regionGoal.center[i] + ((double)rand()/RAND_MAX - 0.5) * (M_PI/180)
				)
      );
      randomStateOut.x[i] = regionGoal.center[i];
		}
	} else {
		for (int i = 0; i < numDimensions; i++) {
			randomStateOut.x[i] = (double)rand()/(RAND_MAX + 1.0)*regionOperating.size[i] 
				- regionOperating.size[i]/2.0 + regionOperating.center[i];
		}
	}

	if (IsInCollision (randomStateOut.x)){
		return 0;
	}

	return 1;
}

#ifdef FK
/*
	 local function get_distance(self, trGoal, qArm, qWaist)
	 -- Grab our relative transform from here to the goal
	 local fkArm = self.forward(qArm, qWaist)
	 local invArm = T.inv(fkArm)
	 local here = trGoal * invArm -- new good one
	 -- Determine the position and angular velocity target
	 local dp = T.position(here)
	 local drpy = T.to_rpy(here)
	 -- TODO: Add the rpy check for other rotation directions
	 return dp, drpy
	 end
	 */

bool System::isReachingTarget (State &stateIn) {
	static const double qWaist[] = {0,0};
	static double xyzS[] = {0,0,0};
  
  /*
  printf("qGuess:\n");
  for (int i=0;i<7;i++){ printf("%f\t", stateIn.x[i]); }
  printf("\n");
  */

	Transform trS = fkLeft7(stateIn.x, qWaist, 0.235,0,0);
	trS.getXYZ(xyzS);

  /*
  printf("trS:\n");
  printTransform(trS);
  printf("trG:\n");
  printTransform(trG);
  */

	Transform here = inv(trS) * trG;
	std::vector<double> dtr6 = position6D(here);

  /*
	 printf("dtr6: ");
	 for (int i=0;i<6;i++){ printf("%f\t", dtr6[i]); }
	 printf("\n");
  */

	for (int i = 0; i < 3; i++) {
		if (fabs(dtr6[i]) > 0.04 ) { return false; }
		//if (fabs(dtr6[i+3]) > 30*M_PI/180 ) { return false; }
		if (fabs(dtr6[i+3]) > 20*M_PI/180 ) { return false; }
	}

  /*
	printf("dp %.2f %.2f %.2f\n", dtr6[0], dtr6[1], dtr6[2]);
	printf("dr %.2f %.2f %.2f\n", dtr6[3]*180/M_PI, dtr6[4]*180/M_PI, dtr6[5]*180/M_PI);
  */

	return true;
}
#else
bool System::isReachingTarget (State &stateIn) {

	for (int i = 0; i < numDimensions; i++) {
		double dist = stateIn.x[i] - regionGoal.center[i];
#ifdef JANGLES
		if (dist > M_PI){
			dist = 2*M_PI - dist;
		} else if(dist < -M_PI){
			dist = 2*M_PI + dist;
		}
#endif
		if (fabs(dist) > regionGoal.size[i]/2.0 ) {
			return false;
		}
	}

	return true;
}
#endif

#ifdef FK
bool System::IsInCollision (double *stateIn) {
	static double qWaist[] = {0,0};
	static double WALL_Y = 0.4;

	double xyz[] = {0,0,0};
	Transform tr = fkLeft7(stateIn, qWaist, 0.235,0,0);
	tr.getXYZ(xyz);
	if (xyz[1] > WALL_Y) {
		//printf("xyz: (%5.2f, %5.2f, %5.2f)\n", xyz[0], xyz[1], xyz[2]);
		return true;
	}
	// Elbow
	fkLeftElbow7(stateIn, qWaist).getXYZ(xyz);
	if (xyz[1] > WALL_Y) {
		printf("elbow: (%5.2f, %5.2f, %5.2f)\n", xyz[0], xyz[1], xyz[2]);
		return true;
	}
	return false;
}
#else
bool System::IsInCollision (double *stateIn) {
	for (list<region*>::iterator iter = obstacles.begin(); iter != obstacles.end(); iter++) {

		region *obstacleCurr = *iter;
		bool collisionFound = true;

		for (int i = 0; i < numDimensions; i++) {
			double dist = obstacleCurr->center[i] - stateIn[i];
#ifdef JANGLES
			// Immaterial, since collisions are kinematic, not really angular...
			if (dist > M_PI){
				dist = 2*M_PI - dist;
			} else if(dist < -M_PI){
				dist = 2*M_PI + dist;
			}
#endif
			if (fabs(dist) > obstacleCurr->size[i]/2.0 ) {
				collisionFound = false;
				break;
			}
		}
		if (collisionFound) {
			return true;
		}
	}

	return false;
}
#endif

double System::distState(double* a, double* b, double *dists) {
	// double *dists = new double[numDimensions];
	for (int i = 0; i < numDimensions; i++) {
		dists[i] = b[i] - a[i];
#ifdef JANGLES
		if (dists[i] > M_PI){
			dists[i] = 2*M_PI - dists[i];
		} else if(dists[i] < -M_PI){
			dists[i] = 2*M_PI + dists[i];
		}
#endif 
	}
	double distTotal = 0.0;
	for (int i = 0; i < numDimensions; i++) {
		distTotal += dists[i]*dists[i];
	}
	distTotal = sqrt (distTotal);
#ifdef JANGLES
	if (distTotal > M_PI){
		distTotal = fabs(2*M_PI - distTotal);
	} else if(distTotal < -M_PI){
		distTotal = fabs(2*M_PI + distTotal);
	}
#endif
	return distTotal;
}

int System::extendTo(State &stateFromIn, State &stateTowardsIn, Trajectory &trajectoryOut, bool &exactConnectionOut)
{

	double * dists = new double[numDimensions];
	double distTotal = System::distState(stateFromIn.x, stateTowardsIn.x, dists);

	double incrementTotal = distTotal / DISCRETIZATION_STEP;

	// normalize the distance according to the discretization step
	for (int i = 0; i < numDimensions; i++){
		dists[i] /= incrementTotal;
	}

	int numSegments = (int)floor(incrementTotal);

	double *stateCurr = new double[numDimensions];
	for (int i = 0; i < numDimensions; i++) {
		stateCurr[i] = stateFromIn.x[i];
	}

	for (int i = 0; i < numSegments; i++) {

		if (IsInCollision (stateCurr))  {
			return 0;
		}

		for (int i = 0; i < numDimensions; i++){
			stateCurr[i] += dists[i];
		}   
	}

	if (IsInCollision (stateTowardsIn.x)){
		return 0;
	}

	trajectoryOut.endState = new State (stateTowardsIn);
	trajectoryOut.totalVariation = distTotal;

	delete [] dists;
	delete [] stateCurr;

	exactConnectionOut = true;

	return 1;
}

double System::evaluateExtensionCost (State& stateFromIn, State& stateTowardsIn, bool &exactConnectionOut) {

	exactConnectionOut = true;

	double * dists = new double[numDimensions];
	return System::distState(stateFromIn.x, stateTowardsIn.x, dists);

}

double System::evaluateCostToGo (State& stateIn) {
	if(regionGoal.getNumDimensions()==0){
		const double qWaist[] = {0,0};
		double xyzS[] = {0,0,0, 0,0,0};

		Transform trS = fkLeft7(stateIn.x, qWaist, 0.235,0,0);
		trS.getXYZ(xyzS);

		Transform here = inv(trS) * trG;
		std::vector<double> dtr6 = position6D(here);

		double distTotal = 0;
		for (int i = 0; i < 3; i++) {
			double dist = dtr6[i];
			distTotal += dist*dist;
		}
		// 1 inch is close
		return sqrt(distTotal) - 0.0254;
	} else {
		double * dists = new double[numDimensions];
		double dist = System::distState(regionGoal.center, stateIn.x, dists);

		double radius = 0.0;
		for (int i = 0; i < numDimensions; i++) {
			radius += regionGoal.size[i] * regionGoal.size[i];
		}
		radius = sqrt(radius);

		return dist - radius;
		//return max(dist - radius, 0); // maybe?
	}

}

