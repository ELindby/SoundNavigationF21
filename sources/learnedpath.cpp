#include "../includes/learnedpath.h"
//Learned path --------------------------------------
LearnedPath::LearnedPath()
{
}

LearnedPath::~LearnedPath()
{
}

void LearnedPath::trackPath(double left_motor_command_, double right_motor_command_, double angle_to_sound_, double angle_to_obst_, double dist_to_obst_)
{
    left_motor_command.push_back(left_motor_command_);
    right_motor_command.push_back(right_motor_command_);
    angle_to_sound.push_back(angle_to_sound_);
    angle_to_obst.push_back(angle_to_obst_);
    dist_to_obst.push_back(dist_to_obst_);
	timesteps_tracked++;
}


//Learned path handler -------------------------------
LearnedPathHandler::LearnedPathHandler()
{
    active_path = new LearnedPath();
}

LearnedPathHandler::~LearnedPathHandler()
{
}

void LearnedPathHandler::handlerTrackPath(double left_motor_command_, double right_motor_command_, double angle_to_sound_, double angle_to_obst_, double dist_to_obst_)
{
	active_path->trackPath(left_motor_command_, right_motor_command_, angle_to_sound_, angle_to_obst_, dist_to_obst_);
}

void LearnedPathHandler::startNewPath()
{
    learned_paths.push_back(*active_path);
    active_path = new LearnedPath();
}

void LearnedPathHandler::getLearnedCommands(const int timestep, double & left_motor_command_o, double & right_motor_command_o){
    left_motor_command_o = learned_paths[0].left_motor_command[timestep];
    right_motor_command_o = learned_paths[0].right_motor_command[timestep];
}
