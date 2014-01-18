/*
 * Dynamics.cpp
 *
 *  Created on: 2013. 12. 4.
 *      Author: hjsong
 */
#include "framework/motion/dynamics.h"

using namespace Thor;

Dynamics* Dynamics::m_UniqueInstance = new Dynamics();

////for default
//double Dynamics::Mass[22] = {0.165, 1.122, 3.432, 2.464, 0.946, 1.133, // Mass of Each Right Leg Part
//		0.165, 1.122, 3.432, 2.464, 0.946, 1.133, // Mass of Each Left Leg Part
//		3.179, 0.13*1.1, 0.81*1.1, 1.067, // Mass of Each Right Arm Part
//		3.179, 0.13*1.1, 0.81*1.1, 1.067, // Mass of Each Left Arm Part
//		8.8, 10.131}; // Mass of Each Body Part

//for drc
double Dynamics::Mass[22] = {0.165, 1.122, 3.432, 2.464, 0.946, 1.133, // Mass of Each Right Leg Part
		0.165, 1.122, 3.432, 2.464, 0.946, 1.133, // Mass of Each Left Leg Part
		3.179, 0.13*1.1, 0.81*1.1, 1.067, // Mass of Each Right Arm Part
		3.179, 0.13*1.1, 0.81*1.1, 1.067, // Mass of Each Left Arm Part
		8.8, 10.131}; // Mass of Each Body Part

Dynamics::Dynamics()
{
//	Mass[0]  = 0.165; Mass[1]  = 1.122;    Mass[2]  = 3.432;    Mass[3] = 2.464;  Mass[4]  = 0.946; Mass[5]  = 1.133; // Mass of Each Right Leg Part
//	Mass[6]  = 0.165; Mass[7]  = 1.122;    Mass[8]  = 3.432;    Mass[9] = 2.464;  Mass[10] = 0.946; Mass[11] = 1.133; // Mass of Each Left Leg Part
//	Mass[12] = 3.179; Mass[13] = 0.13*1.1; Mass[14] = 0.81*1.1; Mass[15] = 1.067; // Mass of Each Right Arm Part
//	Mass[16] = 3.179; Mass[17] = 0.13*1.1; Mass[18] = 0.81*1.1; Mass[19] = 1.067; // Mass of Each Left Arm Part
//	Mass[20] = 8.8;   Mass[21] = 10.131; // Mass of Each Body Part
}

Dynamics::~Dynamics()
{

}
