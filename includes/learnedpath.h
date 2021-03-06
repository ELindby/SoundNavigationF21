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
 #include <stdlib.h>

class LearnedPath
{
private:

public:
    std::vector<double> left_motor_command;
    std::vector<double> right_motor_command;
    std::vector<int> angle_to_sound;

    std::vector<double> angle_to_obst;
    std::vector<double> dist_to_obst;

	int timesteps_tracked = 0;

	LearnedPath();
	~LearnedPath();

	void trackPath(double left_motor_command_, double right_motor_command_, int angle_to_sound_, double angle_to_obst_, double dist_to_obst_);
};

class LearnedPathHandler
{
private:


public:
    LearnedPathHandler();
	~LearnedPathHandler();

    std::vector<LearnedPath> learned_paths;
	LearnedPath * active_path;

	void handlerTrackPath(double left_motor_command_, double right_motor_command_, int angle_to_sound_, double angle_to_obst_, double dist_to_obst_);

	void startNewPath();

	void getLearnedCommands(const int timestep, double & left_motor_command_o, double & right_motor_command_o,
                            const int last_known_angle_to_sound, const double angle_to_obst_, const double dist_to_obst_);
};
