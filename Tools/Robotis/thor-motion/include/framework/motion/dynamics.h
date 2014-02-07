/*
 * Dynamics.h
 *
 *  Created on: 2013. 12. 4.
 *      Author: hjsong
 */

#ifndef DYNAMICS_H_
#define DYNAMICS_H_

namespace Thor
{
class Dynamics
{

private:
	Dynamics();
	static Dynamics* m_UniqueInstance;

public:
	// the Leg is divided to six parts.
	// the arm is divided to four parts.
	// the body is divided to two parts.
	static double Mass[22];
	//		static double Mass[22] = {0.165, 1.122, 3.432, 2.464, 0.946, 1.133, // Mass of Each Right Leg Part
	//										0.165, 1.122, 3.432, 2.464, 0.946, 1.133, // Mass of Each Left Leg Part
	//										3.179, 0.13*1.1, 0.81*1.1, 1.067, // Mass of Each Right Arm Part
	//										3.179, 0.13*1.1, 0.81*1.1, 1.067, // Mass of Each Left Arm Part
	//										8.8, 10.131}; // Mass of Each Body Part

	~Dynamics();

public:
	static Dynamics* GetInstance()			{ return m_UniqueInstance; }
};
}


#endif /* DYNAMICS_H_ */
