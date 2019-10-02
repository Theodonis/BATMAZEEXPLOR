/*
 * FSM.h
 *
 *  Created on: 14.11.2018
 *      Author: cc-isn
 */

#ifndef SOURCES_FSM_H_
#define SOURCES_FSM_H_

/*
 * Define the size of the maze here:
 */
#define X 9
#define Y 9

typedef enum state_ {
	INIT = 1,
	North,
	East,
	South,
	West,
	Drive,
	New_Field,
	Turn_Right,
	Turn_Left,
	NOT_AUS = 99
} state_t;


typedef struct MAZE_data_{

	struct{
		uint8_t maze[X][Y];	/* Input	: */
	}maze_Values;
	//
	struct{
		uint8_t maze_x;
		uint8_t maze_y;
		uint8_t maze_counter;
		uint8_t maze_direction;
	} maze_Parameter;
	//
}MAZE_data_t;


typedef struct MAZE_track_{
		bool wall_left;
		bool wall_right;
		bool wall_front;
		bool wall_back;
		uint16_t fill_Number;
}MAZE_track_t;


/*
 * Methode regelt den ganzen Ablauf.
 * Muss zyklisch aufegrufen werden.
 */
void stateMachine(void);


bool driveSegment(int Segment);


/*
 *
 */
void setState(state_t );

void increaseValue (uint8_t *value);
void decreaseValue (uint8_t *value);

/*
 *
 */
state_t get_actState(void);

/*
 *
 */
MAZE_data_t get_Maze(void);

#endif /* SOURCES_FSM_H_ */
