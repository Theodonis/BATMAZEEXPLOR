/*
 * DrivinExplore_Inreface.h
 *
 *  Created on: 07.11.2019
 *      Author: Theo
 */

#ifndef SOURCES_DRIVINGEXPLORE_INTERFACE_H_
#define SOURCES_DRIVINGEXPLORE_INTERFACE_H_

typedef struct posEstimation{
	float xPos; //(m)
	float yPos; //(m)
	float thetaAngle; //(rad)
}t_PosEstimation;

typedef struct velocEstimation{
	float forwardVeloc; /* forward velocity (m/s)*/
	float angleVeloc;	/* angular velocity (rad/s)*/
}t_VelocEstimation;


typedef struct datameasurement_Driving_to_Explore {
	/* the x-,y-position and the theat-angle {m,m,rad}, estimated from driving control */
	t_PosEstimation posEstimation;
	/* the velocity {forward (m/s), angular (rad/s)}, estimated from driving control */
	t_VelocEstimation velocityEstimation;
}t_data_for_exploration;



#endif /* SOURCES_DRIVINGEXPLORE_INTERFACE_H_ */
