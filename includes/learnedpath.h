#pragma once
/*
 * Description:     Class that contains a learned path towards a sound source
 *                  On every iteration (Reset from goal to start), a new path should be created.
 *
 * Author:			Erik Lindby
 *					University of Southern Denmark
 * Creation date:   22-04-2021
 */

 #include <vector>

class LearnedPath
{
private:
    std::vector<double> left_motor_command;
    std::vector<double> right_motor_command;
    std::vector<double> angle_to_sound;

    std::vector<double> angle_to_obst;
    std::vector<double> dist_to_obst;


public:
	LearnedPath();
	~LearnedPath();

	void trackPath(double left_motor_command_, double right_motor_command_, double angle_to_sound_, double angle_to_obst_, double dist_to_obst_);
};

class LearnedPathHandler
{
private:
    LearnedPath * active_path;
    std::vector<LearnedPath> learned_paths;
public:
    LearnedPathHandler();
	~LearnedPathHandler();

	void startNewPath();
};
