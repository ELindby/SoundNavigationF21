#include "../includes/learnedpath.h"
//Learned path --------------------------------------
LearnedPath::LearnedPath()
{
}

LearnedPath::~LearnedPath()
{
}

void LearnedPath::trackPath(double left_motor_command_, double right_motor_command_, int angle_to_sound_, double angle_to_obst_, double dist_to_obst_)
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

void LearnedPathHandler::handlerTrackPath(double left_motor_command_, double right_motor_command_, int angle_to_sound_, double angle_to_obst_, double dist_to_obst_)
{
	active_path->trackPath(left_motor_command_, right_motor_command_, angle_to_sound_, angle_to_obst_, dist_to_obst_);
}

void LearnedPathHandler::startNewPath()
{
    learned_paths.push_back(*active_path);
    active_path = new LearnedPath();
}

void LearnedPathHandler::getLearnedCommands(const int timestep, double & left_motor_command_o, double & right_motor_command_o,
                                            const int last_known_angle_to_sound, const double angle_to_obst_, const double dist_to_obst_){
    //Find the learned path with closest parameters to the actual scenario
    //NOTE: This can be vastly improved with a localization method ike extended kalman filter,
    //or a particle filter, but this wasn't implemented because of a lack of time
    double path_unlikeness      = abs(last_known_angle_to_sound - learned_paths[0].angle_to_sound[0])/ 360   * 10
                                + abs(angle_to_obst_            - learned_paths[0].angle_to_obst[0]) / 360   * 10
                                + abs(dist_to_obst_             - learned_paths[0].dist_to_obst[0])  / 10000 * 10;
    int closest_path_index = 0;
    int closest_path_timestep = 0;

    double path_unlikeness_temp;
    for(size_t i = 1; i < learned_paths.size(); i++){
        for(int j = 0; j < learned_paths[i].timesteps_tracked; j++){
            path_unlikeness_temp    = abs(last_known_angle_to_sound - learned_paths[i].angle_to_sound[j])/ 360   * 10
                                    + abs(angle_to_obst_            - learned_paths[i].angle_to_obst[j])    / 360   * 10
                                    + abs(dist_to_obst_             - learned_paths[i].dist_to_obst[j])     / 10000 * 10;
            if(path_unlikeness_temp < path_unlikeness){
                closest_path_index = i;
                closest_path_timestep = j;
                path_unlikeness = path_unlikeness_temp;
            }
        }
    }
    left_motor_command_o = learned_paths[closest_path_index].left_motor_command[closest_path_timestep];
    right_motor_command_o = learned_paths[closest_path_index].right_motor_command[closest_path_timestep];
}
