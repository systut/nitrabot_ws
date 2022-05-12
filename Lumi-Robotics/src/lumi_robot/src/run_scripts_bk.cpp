// C program to illustrate
// pipe system call in C
#include <stdio.h>
#include<iostream>
#include <unistd.h>
#include "ros/ros.h"
#define MSGSIZE 16

const char* startRobot = "gnome-terminal -- bash -x 'cd ~/Lumi-Robots; bash start_nodes.sh'";
const char* startNavigationCommand = "cd ~/Lumi-Robots; bash start_move_base.sh";
const char* startCartographerOnlyLidar = "cd ~/catkin_ws; bash start_slam_only_lidar_phase2.sh";
const char* startCartographerPureLocalization = "cd ~/catkin_ws; bash start_pure_localization.sh";
const char* startFinderNode = "cd ~/Lumi-Robotics; bash start_finder_node.sh" ;
const char* startRecorderNode = "cd ~/Lumi-Robotics; bash start_record_node.sh" ;
const char* startCreatingMapCommand = "cd ~/Lumi-Robotics; bash start_creating_map.sh" ;

void startMapping()
{
    char inbuf[MSGSIZE];

    int p[2], i;
  
    if (pipe(p) < 0)
        std::exit(1);
  
    /* continued */
    /* write pipe */
  
    write(p[1], startCartographerOnlyLidar, MSGSIZE);
  
    for (i = 0; i < 3; i++) {
        /* read pipe */
        read(p[0], inbuf, MSGSIZE);
        printf("% s\n", inbuf);
    }
}
 
void startCreatingMap()
{
    char inbuf[MSGSIZE];

    int p[2], i;
  
    if (pipe(p) < 0)
        std::exit(1);
  
    /* continued */
    /* write pipe */
  
    write(p[1], startRobot, MSGSIZE);
  
    for (i = 0; i < 3; i++) {
        /* read pipe */
        read(p[0], inbuf, MSGSIZE);
        printf("% s\n", inbuf);
    }
}

void startRecordingPath()
{
    char inbuf[MSGSIZE];

    int p[2], i;
  
    if (pipe(p) < 0)
        std::exit(1);
  
    /* continued */
    /* write pipe */
  
    write(p[1], startCartographerPureLocalization, MSGSIZE);
    write(p[1], startRecorderNode, MSGSIZE);
  
    for (i = 0; i < 2; i++) {
        /* read pipe */
        read(p[0], inbuf, MSGSIZE);
        printf("% s\n", inbuf);
    }
}

void startNavigation()
{
    char inbuf[MSGSIZE];

    int p[2], i;
  
    if (pipe(p) < 0)
        std::exit(1);
  
    /* continued */
    /* write pipe */
  
    write(p[1], startCartographerPureLocalization, MSGSIZE);
    write(p[1], startNavigationCommand, MSGSIZE);
    write(p[1], startFinderNode, MSGSIZE);
  
    for (i = 0; i < 3; i++) {
        /* read pipe */
        read(p[0], inbuf, MSGSIZE);
        printf("% s\n", inbuf);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_finder_node");

    ros::NodeHandle nh;
    
    startCreatingMap();
        
    return 0;
}

