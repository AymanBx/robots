/*-------------------------------------------------------------------------+
|	Final Project CSC412 - Spring 2023										|
|	A graphic front end for a box-pushing simulation.						|
|																			|
|	This application simply creates a glut window with a pane to display	|
|	a colored grid and the other to display some state information.			|
|																			|
|	Current GUI:															|
|		- 'ESC' --> exit the application									|
|		- ',' --> slows down the simulation									|
|		- '.' --> apeeds up  he simulation									|
|																			|
|	Created by Jean-Yves Hervé on 2018-12-05 (C version)					|
|	Revised 2023-04-27														|
+-------------------------------------------------------------------------*/
//
//  main.cpp
//  Final Project CSC412 - Spring 2023
//
//  Created by Jean-Yves Hervé on 2018-12-05, Rev. 2023-12-01
//	This is public domain code.  By all means appropriate it and change is to your
//	heart's content.
#include <iostream>
#include <sstream>
#include <fstream>
#include <string.h>
#include <random>
#include <thread>
#include <vector>
#include <map>
#include <mutex>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
//
//
#include "glPlatform.h"
#include "typesAndConstants.h"
#include "gl_frontEnd.h"

using namespace std;

#if 0
//=================================================================
#pragma mark -
#pragma mark Function prototypes
//=================================================================
#endif

void displayGridPane(void);
void displayStatePane(void);
void printHorizontalBorder(ostringstream &outStream);
string printGrid(void);
void initializeApplication(void);
void cleanupAndQuit();
void generatePartitions();
// Added functions
void initDoors();
void initRobots();
void initBoxes();
void removeDoors();
bool checkAvailability(GridPosition coordinates);
bool checkRobots(GridPosition coordinates);
bool checkBoxes(GridPosition coordinates);
bool checkMovement(GridPosition coordinates);
GridPosition getDistance(GridPosition mover, GridPosition destination);
// void move(Robot* robot, Direction dir);
// void push(Robot* robot, Direction dir);

#if 0
//=================================================================
#pragma mark -
#pragma mark Application-level global variables
//=================================================================
#endif

//	Don't touch
extern int gMainWindow, gSubwindow[2];

//-------------------------------------
//	Don't rename any of these variables
//-------------------------------------
//	The state grid's dimensions (arguments to the program)
int numRows = -1;  //	height of the grid
int numCols = -1;  //	width
int numBoxes = -1; //	also the number of robots
int numDoors = -1; //	The number of doors.

int numLiveThreads = 0; //	the number of live robot threads

//	robot sleep time between moves (in microseconds)
int robotSleepTime = 1000000;

//	An array of C-string where you can store things you want displayed
//	in the state pane to display (for debugging purposes?)
//	Dont change the dimensions as this may break the front end
const int MAX_NUM_MESSAGES = 8;
const int MAX_LENGTH_MESSAGE = 32;
char **message;

//	Only absolutely needed if you tackle the partition EC
SquareType **grid;

//-----------------------------
//	CHANGE THIS
//-----------------------------
//	Here I hard-code myself some data for robots and doors.  Obviously this code
//	must go away.  I just want to show you how information gets displayed.
//	Obviously, you will need to allocate yourself some dynamic data structure to store
//	that information once you know the dimensions of the grid, number of boxes/robots and
//	doors.
//	Note that, even if you use the GUI version, it doesn't impose you a storage format at
//	all, since the drawing function draw a single robot/box/door at a time, and take that
//	object's parameters as individual arguments.
//	So, feel free to go vectors all the way if you like it better than int**
//	Just don't feel free to declare oversized arrays, in the style of
//	int robotLoc[1000][2];
//	I can guarantee you that this kind of stuff will get penalized harshly (it might have
//	been convenient, borderline cute, in CSC211, but by now it's absolutely embarrassing)
//
//	Also:  Please note that because this is a grid-based problem, I never think of x and y but
//			row and column (well, the "I" dealing with the planning problem.  The "I" doing
//			the dirty work underneath has to translate all of that row and column data into
//			x and y pixel coordinates for OpenGL for rendering
//		   So,... In all of these arrays of 2 int values, the first value (index 0)
//			is a row coordinate and the second value (index 1) is a column coordinate.
// int doorAssign[] = {1, 0, 0, 2, 1, 3}; //	door id assigned to each robot-box pair
// int robotLoc[][2] = {{12, 8}, {6, 9}, {3, 14}, {11, 15}, {14, 1}, {8, 13}};
// int boxLoc[][2] = {{6, 7}, {4, 12}, {13, 13}, {8, 12}, {7, 14}, {11, 9}};
// int doorLoc[][2] = {{3, 3}, {8, 11}, {7, 10}, {12, 6}};

//	The above hard-coded intialization should be replaced by random generation in
//	initializeApplication().
//	Of course, this means that you need to modify the type of the above variables
// int** robotLoc;
// int** boxLoc;
// int** doorAssign;
// int** doorLoc;
//	Or with a bit of retooling
vector<int> doorAssign;
vector<GridPosition> doorLoc;
vector<GridPosition> boxLoc;
vector<Robot> robots;

vector<thread> threads;

// A vector that holds all the filled-up cells
vector<GridPosition> filledCells;

// Random engine init
random_device randDev;
default_random_engine engine(randDev());

// Random ranges
uniform_int_distribution<int> boxRowDist;
uniform_int_distribution<int> boxColDist;
uniform_int_distribution<int> doorDist;
uniform_int_distribution<int> rowDist;
uniform_int_distribution<int> colDist;

// Create & initialize locks
mutex gridLock;
mutex outputLock;

// Annotate the output file
ofstream output("robotSimulOut.txt");

//	For extra credit section
vector<SlidingPartition> partitionList;
//	Change argument to 0.5 for equal probability of vertical and horizontal partitions
//	0 for only horizontal, and 1 for only vertical
bernoulli_distribution headsOrTails(1.0);

#if 0
//=================================================================
#pragma mark -
#pragma mark Function implementations
//=================================================================
#endif

//------------------------------------------------------------------------
//	You shouldn't have to change much in the main function besides
//	the initialization of numRows, numCos, numDoors, numBoxes.
//------------------------------------------------------------------------
int main(int argc, char **argv)
{
	//	We know that the arguments  of the program  are going
	//	to be the width (number of columns) and height (number of rows) of the
	//	grid, the number of boxes (and robots), and the number of doors.
	//	You are going to have to extract these.

	// Error checking for arguments entered by user:
	//
	// if (argc = 3){
	// 	numBoxes = std::atoi(argv[1]);
	// 	numDoors = std::atoi(argv[2]);
	// }
	//
	if (argc != 5){
		cerr<< "Usage: ./robotsV0 <rows> <columns> <boxes> <doors>" << endl;
		exit(404);
	}
	else{
		numRows = std::atoi(argv[1]);
		numCols = std::atoi(argv[2]);
		numBoxes = std::atoi(argv[3]);
		numDoors = std::atoi(argv[4]);
       	cout<< "Num of args = " << argc<< endl;
	}
	cout <<"Check 0" << endl;

	if (numRows*numCols < 6*numBoxes){
		numRows = 2*numBoxes;
		numCols = 2*numBoxes;
	}
	cout <<"Check 1" << endl;

	if (numBoxes < 1){
		cerr<< "You have entered a low number of boxes" << endl;
		exit(404);
	}
    cout <<"Check 2" << endl;

	if (numDoors < 1 || numDoors > 3){
		cerr<< "You have entered a weird number of doors" << endl;
		exit(404);
	}
    cout <<"Check 3" << endl;
	
	// Fill up file header
	output << numRows << " " << numCols << " " << numBoxes << " " << numDoors << " " << endl << endl;

	// Initialize the random ranges
	boxRowDist = uniform_int_distribution<int>(1, numRows - 2);
	boxColDist = uniform_int_distribution<int>(1, numCols - 2);
	doorDist = uniform_int_distribution<int>(0, numDoors - 1);
	rowDist = uniform_int_distribution<int>(1, numRows - 1);
	colDist = uniform_int_distribution<int>(1, numCols - 1);
	

	//	Even though we extracted the relevant information from the argument
	//	list, I still need to pass argc and argv to the front-end init
	//	function because that function passes them to glutInit, the required call
	//	to the initialization of the glut library.
	initializeFrontEnd(argc, argv, displayGridPane, displayStatePane);

	//	Now we can do application-level initialization
	initializeApplication();

	// ********************************* uncomment? *********************************
	// string outStr = printGrid();
	// cout << outStr << endl;

	//	Now we enter the main loop of the program and to a large extend
	//	"lose control" over its execution.  The callback functions that
	//	we set up earlier will be called when the corresponding event
	//	occurs
	glutMainLoop();

	cleanupAndQuit();

	//	This will probably never be executed (the exit point will be in one of the
	//	call back functions).
	return 0;
}

void cleanupAndQuit()
{
	//	//	Free allocated resource before leaving (not absolutely needed, but
	//	//	just nicer.  Also, if you crash there, you know something is wrong
	//	//	in your code.
	
	for(unsigned int i = 0; i < robots.size(); i++){
		robots[i].isAlive = false;
	}
	
	for (unsigned int i = 0; i < threads.size(); i++){
		threads[i].join();
	}

	for (int i = 0; i < numRows; i++)
		delete[] grid[i];
	delete[] grid;
	for (int k = 0; k < MAX_NUM_MESSAGES; k++)
		delete[] message[k];
	delete[] message;


	exit(0);
}

void initializeApplication(void)
{
	//	Allocate the grid
	grid = new SquareType *[numRows];
	for (int i = 0; i < numRows; i++)
		grid[i] = new SquareType[numCols];

	message = new char *[MAX_NUM_MESSAGES];
	for (int k = 0; k < MAX_NUM_MESSAGES; k++)
		message[k] = new char[MAX_LENGTH_MESSAGE + 1];

	//---------------------------------------------------------------
	//	This is the place where you should initialize the location
	//	of the doors, boxes, and robots, and create threads (not
	//	necessarily in that order).
	//---------------------------------------------------------------
	

	doorAssign.clear();
	initDoors();
	initBoxes();
	initRobots();

	//	For extra credit
	// generatePartitions();
}

vector<int> directionsToBoxPushStart(Robot* robot){
	
	vector<int> ops;

	GridPosition box = boxLoc[robot->num];
	GridPosition door = doorLoc[robot->assignedDoor];
	
	GridPosition boxDistance = getDistance(box, door);
	
	GridPosition robotPushingPosH;
	robotPushingPosH.row = box.row;
	if (boxDistance.col < 0){
		robotPushingPosH.col = box.col + 1;
	}
	else {
		robotPushingPosH.col = box.col - 1;
	}
	
	
	GridPosition robotDistanceH = getDistance(robot->coordinates, robotPushingPosH);
	
	bool left = robotDistanceH.col < 0 ? true : false;
	bool up = robotDistanceH.row < 0 ? true : false;
	
	for(int i = 0; i < abs(robotDistanceH.col); i++){
		if(left){
			ops.push_back(0);
		}
		else{
			ops.push_back(2);
		}
	}
	
	for(int i = 0; i < abs(robotDistanceH.row); i++){
		if(up){
			ops.push_back(1);
		}
		else{
			ops.push_back(3);
		}
	}
	
	return ops;
}

vector<int> directionsToVerticalToDoor(Robot* robot){

	vector<int> ops;

	GridPosition box = boxLoc[robot->num];
	GridPosition door = doorLoc[robot->assignedDoor];
	
	GridPosition boxDistance = getDistance(box, door);
	
	GridPosition robotPushingPosV;
	robotPushingPosV.col = door.col;
	if (boxDistance.row > 0){
		robotPushingPosV.row = box.row + 1;
	}
	else {
		robotPushingPosV.row = box.row - 1;
	}
	
	
	GridPosition robotDistanceV = getDistance(box, robotPushingPosV);
	
	
	bool left = robotDistanceV.col < 0 ? true : false;
	bool up = robotDistanceV.row < 0 ? true : false;
	
	// Will move to previous function with boxDistance instead...
	for(int i = 0; i < abs(robotDistanceV.col); i++){
		if(left){
			ops.push_back(4);
		}
		else{
			ops.push_back(6);
		}
	}

	if(up){
		ops.push_back(3);
	}
	else{
		ops.push_back(1);
	}
	
	if(left){
		ops.push_back(0);
	}
	else{
		ops.push_back(2);
	}
	
	return ops;
}

vector<int> directionsToDoor(Robot* robot){

	vector<int> ops;
	
	GridPosition box = boxLoc[robot->num];
	GridPosition door = doorLoc[robot->assignedDoor];
	
	GridPosition toDoor = getDistance(box, door);
	
	
	bool up = toDoor.row < 0 ? true : false;
	
	for(int i = 0; i < abs(toDoor.row) + 1; i++){
		if(up){
			ops.push_back(5);
		}
		else{
			ops.push_back(7);
		}
	}
	
	return ops;
}

vector<int> path_box_to_door(Robot* robot){

	vector<int> ops;
	vector<int> dbs = directionsToBoxPushStart(robot);
	vector<int> dvd = directionsToVerticalToDoor(robot);
	vector<int> dtd = directionsToDoor(robot);
	
	ops.insert(end(ops), begin(dbs), end(dbs));
	ops.insert(end(ops), begin(dvd), end(dvd));
	ops.insert(end(ops), begin(dtd), end(dtd));
	
	return ops;
}

void end(Robot* robot){
	robot->isAlive = false;
	robot->coordinates = {-1, -1};
	boxLoc[robot->num] = {-1, -1};
}

// multithreaded robots
// Argument here was void, but I wanted to pass the robot as an argument
void robotFunc(Robot* robot)
{
	
	bool isAlive = true;
	unsigned int index = 0;
	
	
	// 0 left, 1 up, 2 right, 3 down
	// 4 push left, 5 push up, 6 push right, 7 push down
	vector<int> ops = path_box_to_door(robot);
	
	while(isAlive){
		
		usleep(robotSleepTime / 10);
		
		GridPosition toMove;
		GridPosition boxMove;
		
		// 0 left, 1 up, 2 right, 3 down 
		// 4 push left, 5 push up, 6 push right, 7 push down
		// 8 repath
		switch(ops[index]){

			case 0: // left move command
			toMove.row = robot->coordinates.row;
			toMove.col = robot->coordinates.col - 1;
			
			gridLock.lock();

			if(!checkMovement(toMove)){
				gridLock.unlock();
				continue;
			}
			
			robot->coordinates = toMove;
			gridLock.unlock();
			
			outputLock.lock();
			output << "robot " << robot->num << " move W" << endl;
			outputLock.unlock();
			
			index++;
			break;


			case 2: // right move command
			toMove.row = robot->coordinates.row;
			toMove.col = robot->coordinates.col + 1;
			
			gridLock.lock();
			// If move right failed: move robot left then up then repath
			if(!checkMovement(toMove)){
				gridLock.unlock();
				continue;
			}
			
			robot->coordinates = toMove;
			gridLock.unlock();
			
			outputLock.lock();
			output << "robot " << robot->num << " move E" << endl;
			outputLock.unlock();
			
			index++;
			break;


			case 1: // move up command
			
			toMove.row = robot->coordinates.row - 1;
			toMove.col = robot->coordinates.col;
			
			gridLock.lock();
			// If move up failed: move robot down then right then repath
			if(!checkMovement(toMove)){
				gridLock.unlock();
				continue;
			}
			
			robot->coordinates = toMove;
			gridLock.unlock();
			
			outputLock.lock();
			output << "robot " << robot->num << " move N" << endl;
			outputLock.unlock();
			
			index++;
			break;


			case 3: // move down command
			
			toMove.row = robot->coordinates.row + 1;
			toMove.col = robot->coordinates.col;
			
			gridLock.lock();
			// If move down failed: move robot up then left then repath
			if(!checkMovement(toMove)){
				gridLock.unlock();
				continue;
			}
			
			robot->coordinates = toMove;
			gridLock.unlock();
			
			outputLock.lock();
			output << "robot " << robot->num << " move S" << endl;
			outputLock.unlock();
			
			index++;
			break;


			case 4: // left push command
			
			toMove.row = robot->coordinates.row;
			toMove.col = robot->coordinates.col - 1;
			boxMove.row = boxLoc[robot->num].row;
			boxMove.col = boxLoc[robot->num].col - 1;
			
			gridLock.lock();
			// If failed, repeat
			if(!checkMovement(boxMove) && !(boxMove == doorLoc[robot->assignedDoor])){
				gridLock.unlock();
				continue;
			}
			
			robot->coordinates = toMove;
			boxLoc[robot->num] = boxMove;
			gridLock.unlock();
			
			outputLock.lock();
			output << "robot " << robot->num << " push W" << endl;
			outputLock.unlock();
			
			index++;
			break;


			case 5: // up push command
			
			toMove.row = robot->coordinates.row - 1;
			toMove.col = robot->coordinates.col;
			boxMove.row = boxLoc[robot->num].row - 1;
			boxMove.col = boxLoc[robot->num].col;
			
			gridLock.lock();
			// If failed, repeat
			if(!checkMovement(boxMove) && !(boxMove == doorLoc[robot->assignedDoor])){
				gridLock.unlock();
				continue;
			}
			
			robot->coordinates = toMove;
			boxLoc[robot->num] = boxMove;
			gridLock.unlock();
			
			outputLock.lock();
			output << "robot " << robot->num << " push N" << endl;
			outputLock.unlock();
			
			index++;
			break;


			case 6: // right push command
			
			toMove.row = robot->coordinates.row;
			toMove.col = robot->coordinates.col + 1;
			boxMove.row = boxLoc[robot->num].row;
			boxMove.col = boxLoc[robot->num].col + 1;
			
			gridLock.lock();
			// If failed, repeat
			if(!checkMovement(boxMove) && !(boxMove == doorLoc[robot->assignedDoor])){
				gridLock.unlock();
				continue;
			}
			
			robot->coordinates = toMove;
			boxLoc[robot->num] = boxMove;
			gridLock.unlock();
			
			outputLock.lock();
			output << "robot " << robot->num << " push E" << endl;
			outputLock.unlock();
			
			index++;
			break;


			case 7: // down push command
			
			toMove.row = robot->coordinates.row + 1;
			toMove.col = robot->coordinates.col;
			boxMove.row = boxLoc[robot->num].row + 1;
			boxMove.col = boxLoc[robot->num].col;
			
			gridLock.lock();
			// If failed, repeat
			if(!checkMovement(boxMove) && !(boxMove == doorLoc[robot->assignedDoor])){
				gridLock.unlock();
				continue;
			}
			
			robot->coordinates = toMove;
			boxLoc[robot->num] = boxMove;
			gridLock.unlock();
			
			outputLock.lock();
			output << "robot " << robot->num << " push S" << endl;
			outputLock.unlock();
			
			index++;
			break;
		}
		
		if(index == ops.size() || boxLoc[robot->num] == doorLoc[robot->assignedDoor] || robot->isAlive == false){
			end(robot);
			break;
		}
		
	}
}

bool checkRobots(GridPosition coordinates){
	for (unsigned int i = 0; i < robots.size(); i++){
		if (coordinates == robots[i].coordinates && robots[i].isAlive){
			return false;
		}
	}
	return true;
}

bool checkBoxes(GridPosition coordinates){
	for (unsigned int i = 0; i < boxLoc.size(); i++){
		if (coordinates == boxLoc[i]){
			return false;
		}
	}
	return true;
}

bool checkMovement(GridPosition coordinates){
	return checkRobots(coordinates) && checkBoxes(coordinates);
}

bool checkAvailability(GridPosition coordinates){
	for (unsigned int i = 0; i < filledCells.size(); i++){
		if (coordinates == filledCells[i]){
			return false;
		}
	}
	return true;
}

void removeDoors(){
	for(unsigned int i = 0; i < doorLoc.size(); i++){
		for(unsigned int j = 0; j < filledCells.size(); j++){
			if(doorLoc[i] == filledCells[j]){
				filledCells.erase(filledCells.begin() + j);
				break;
			}
		}
	}
}

void initDoors(){
	while (doorLoc.size() < (unsigned)numDoors){
		int row = rowDist(engine);
		int col = colDist(engine);
		GridPosition coordinates = {row, col};
		bool available = checkAvailability(coordinates);
		if (available){
			output << "door " << doorLoc.size() << ": " << "(" << row << ", " << col << ")" << endl;
			doorLoc.push_back(coordinates);
			filledCells.push_back(coordinates);
		}
	}
	output << endl;
}

void initBoxes(){
	while (boxLoc.size() < (unsigned)numBoxes){
		int row = boxRowDist(engine);
		int col = boxColDist(engine);
		GridPosition coordinates = {row, col};
		bool available = checkAvailability(coordinates);
		if (available){
			output << "box " << boxLoc.size() << ": " << "(" << row << ", " << col << ")" << endl;
			boxLoc.push_back(coordinates);
			filledCells.push_back(coordinates);
		}
	}
	output << endl;
}

void initRobots(){
	while (robots.size() < (unsigned)numBoxes){
		int row = rowDist(engine);
		int col = colDist(engine);
		
		int randDoor = doorDist(engine);
		
		GridPosition coordinates = {row, col};
		bool available = checkAvailability(coordinates);
		if (available){
			Robot robot{
				true,
				(unsigned)robots.size(),
				(unsigned)robots.size(),
				coordinates,
				(unsigned)randDoor
			};
			output << "robot " << robots.size() << ": " << "(" << row << ", " << col << ")" << endl;
			robots.push_back(robot);
			doorAssign.push_back(robot.assignedDoor);
			filledCells.push_back(coordinates);
			
		}
	}
	
	output << endl;

	// Doors don't move on the grid so we can have robots and boxes jump over them.
	removeDoors();
	
	for(unsigned int i = 0; i < robots.size(); i++){
		threads.push_back(thread(robotFunc, &robots[i]));
		// Error checking in case threading failed...
		
		
		if (!threads[i].joinable()){
			cerr << "Something went wrong; robot #" << i+1 << "failed to initialize a thread." << endl;
			exit(404);
		}
		
		
		numLiveThreads++;
		usleep(10000);
	}
}

GridPosition getDistance(GridPosition mover, GridPosition destination){
	GridPosition distance;
	distance.col = destination.col - mover.col;
	distance.row = destination.row - mover.row;
	return distance;
}


//	Rather that writing a function that prints out only to the terminal
//	and then
//		a. restricts me to a terminal-bound app;
//		b. forces me to duplicate the code if I also want to output
//			my grid to a file,
//	I have two options for a "does it all" function:
//		1. Use the stream class inheritance structure (the terminal is
//			an iostream, an output file is an ofstream, etc.)
//		2. Produce an output file and let the caller decide what they
//			want to do with it.
//	I said that I didn't want this course to do too much OOP (and, to be honest,
//	I have never looked seriously at the "stream" class hierarchy), so we will
//	go for the second solution.
string printGrid(void)
{
	//	some ugly hard-coded stuff
	static string doorStr[] = {"D0", "D1", "D2", "D3", "DD4", "D5", "D6", "D7", "D8", "D9"};
	static string robotStr[] = {"r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7", "r8", "r9"};
	static string boxStr[] = {"b0", "b1", "b2", "b3", "b4", "b5", "b6", "b7", "b8", "b9"};

	if (numDoors > 10 || numBoxes > 10)
	{
		cout << "This function only works for small numbers of doors and robots" << endl;
		exit(1);
	}

	//	I use sparse storage for my grid
	map<int, map<int, string>> strGrid;

	//	addd doors
	for (int k = 0; k < numDoors; k++)
	{
		strGrid[doorLoc[k].row][doorLoc[k].col] = doorStr[k];
	}
	//	add boxes
	for (int k = 0; k < numBoxes; k++)
	{
		strGrid[boxLoc[k].row][boxLoc[k].col] = boxStr[k];
		strGrid[robots[k].coordinates.row][robots[k].coordinates.col] = robotStr[k];
	}

	ostringstream outStream;

	//	print top border
	printHorizontalBorder(outStream);

	for (int i = 0; i < numRows; i++)
	{
		outStream << "|";
		for (int j = 0; j < numCols; j++)
		{
			if (strGrid[i][j].length() > 0)
				outStream << " " << strGrid[i][j];
			else
			{
				outStream << " . ";
			}
		}
		outStream << "|" << endl;
	}
	//	print bottom border
	printHorizontalBorder(outStream);

	strGrid.clear();
	return outStream.str();
}

void printHorizontalBorder(ostringstream &outStream)
{
	outStream << "+--";
	for (int j = 1; j < numCols; j++)
	{
		outStream << "---";
	}
	outStream << "-+" << endl;
}

void generatePartitions(void)
{
	const unsigned int NUM_PARTS = (numCols + numRows) / 4;

	//	I decide that a partition length  cannot be less than 3  and not more than
	//	1/4 the grid dimension in its Direction
	const unsigned int MIN_PARTITION_LENGTH = 3;
	const unsigned int MAX_HORIZ_PART_LENGTH = numCols / 4;
	const unsigned int MAX_VERT_PART_LENGTH = numRows / 4;
	const unsigned int MAX_NUM_TRIES = 20;
	uniform_int_distribution<unsigned int> horizPartLengthDist(MIN_PARTITION_LENGTH, MAX_HORIZ_PART_LENGTH);
	uniform_int_distribution<unsigned int> vertPartLengthDist(MIN_PARTITION_LENGTH, MAX_VERT_PART_LENGTH);
	uniform_int_distribution<unsigned int> rowDist(1, numRows - 2);
	uniform_int_distribution<unsigned int> colDist(1, numCols - 2);

	for (unsigned int w = 0; w < NUM_PARTS; w++)
	{
		//	Case of a vertical partition
		if (headsOrTails(engine))
		{
			bool goodPart = false;

			//	I try a few times before giving up
			for (unsigned int k = 0; k < MAX_NUM_TRIES && !goodPart; k++)
			{
				//	let's be hopeful
				goodPart = true;

				//	select a column index
				unsigned int col = colDist(engine);
				unsigned int length = vertPartLengthDist(engine);

				//	now a random start row
				unsigned int startRow = 1 + rowDist(engine) % (numRows - length - 1);
				for (unsigned int row = startRow, i = 0; i < length && goodPart; i++, row++)
				{
					if (grid[row][col] != SquareType::FREE_SQUARE)
						goodPart = false;
				}

				//	if the partition is possible,
				if (goodPart)
				{
					//	add it to the grid and to the partition list
					SlidingPartition part;
					part.isVertical = true;
					for (unsigned int row = startRow, i = 0; i < length && goodPart; i++, row++)
					{
						grid[row][col] = SquareType::VERTICAL_PARTITION;
						GridPosition pos = {(int)row, (int)col};
						part.blockList.push_back(pos);
					}

					partitionList.push_back(part);
				}
			}
		}
		// case of a horizontal partition
		else
		{
			bool goodPart = false;

			//	I try a few times before giving up
			for (unsigned int k = 0; k < MAX_NUM_TRIES && !goodPart; k++)
			{
				//	let's be hopeful
				goodPart = true;

				//	select a row index
				unsigned int row = rowDist(engine);
				unsigned int length = vertPartLengthDist(engine);

				//	now a random start row
				unsigned int startCol = 1 + colDist(engine) % (numCols - length - 1);
				for (unsigned int col = startCol, i = 0; i < length && goodPart; i++, col++)
				{
					if (grid[row][col] != SquareType::FREE_SQUARE)
						goodPart = false;
				}

				//	if the wall first, add it to the grid and build SlidingPartition object
				if (goodPart)
				{
					SlidingPartition part;
					part.isVertical = false;
					for (unsigned int col = startCol, i = 0; i < length && goodPart; i++, col++)
					{
						grid[row][col] = SquareType::HORIZONTAL_PARTITION;
						GridPosition pos = {(int)row, (int)col};
						part.blockList.push_back(pos);
					}

					partitionList.push_back(part);
				}
			}
		}
	}
}
