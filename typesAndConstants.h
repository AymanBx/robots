//
//  typesAndConstants.h
//  GL threads
//
//  Created by Jean-Yves Hervé on 2021-12-07
//

#ifndef TYPES_AND_CONSTANTS_H
#define TYPES_AND_CONSTANTS_H

#include <vector>
#include <thread>

//===============================================
//	Application-wide constants
//===============================================
inline constexpr int 	MIN_SLEEP_TIME = 1000;

inline constexpr int	GRID_PANE_WIDTH = 900,
						GRID_PANE_HEIGHT = GRID_PANE_WIDTH,
						STATE_PANE_WIDTH = 300,
						STATE_PANE_HEIGHT = GRID_PANE_HEIGHT,
						H_PADDING = 5,
						WINDOW_WIDTH = GRID_PANE_WIDTH + STATE_PANE_WIDTH + H_PADDING,
						WINDOW_HEIGHT = GRID_PANE_HEIGHT;


//===============================================
//	Custom data types
//===============================================

//	Travel direction data type
enum Direction {
					NORTH = 0,
					WEST,
					SOUTH,
					EAST,
					//
					NUM_TRAVEL_DIRECTIONS
};

enum FontSize {
					SMALL_FONT_SIZE,
					MEDIUM_FONT_SIZE,
					LARGE_FONT_SIZE,
					//
					NUM_FONT_SIZES
};

enum class SquareType
{
	FREE_SQUARE,
	DOOR,
	ROBOT,
	BOX,
	VERTICAL_PARTITION,
	HORIZONTAL_PARTITION,
	//
	NUM_SQUARE_TYPES

};


enum RobotMove{
	moveHToH = 0,
	moveVToH,
	pushH,
	moveVToV,
	moveHToV,
	pushV,
	//
	typesOfMoves
};


/**	Data type to store the position of *things* on the grid
 */
struct GridPosition
{
	/**	row index
	 */
	int row;
	/** column index
	 */
	int col;

	// Overload the '==' operator for this struct
	bool operator==(const GridPosition& other) const {
    return (row == other.row) && (col == other.col);
  	}

	// Overload the '!=' operator for this struct
  	bool operator!=(const GridPosition& other) const {
    return (row != other.row) || (col != other.col);
  	}

};

/**
 *	Data type to represent a sliding partition
 */
struct SlidingPartition
{
	/*	vertical vs. horizontal partition
	 */
	bool isVertical;

	/**	The blocks making up the partition, listed
	 *		top-to-bottom for a vertical list
	 *		left-to-right for a horizontal list
	 */
	std::vector<GridPosition> blockList;

};



struct Robot
{
	bool isAlive;
	unsigned int num;
	pthread_t thread_id
	GridPosition coordinates;
	unsigned int assignedDoor;
	Direction dir;
	RobotMove moveType;
};

#endif //	TYPES_AND_CONSTANTS_H
