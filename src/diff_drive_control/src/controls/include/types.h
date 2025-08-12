#pragma once

struct Pose2d 
{
    double x;
    double y;
    double heading;
};

struct Pose3d
{
    double x;
    double y;
    double z;
    double heading;
};

enum MoveState
{
    TURN,
    STABILIZE_ANGLE,
    MOVE,
    STOP
};