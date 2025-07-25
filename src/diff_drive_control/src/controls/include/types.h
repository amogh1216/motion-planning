#pragma once

struct Pose2d 
{
    double x;
    double y;
    double heading;
};

enum MoveState
{
    TURN,
    STABILIZE_ANGLE,
    MOVE,
    STOP
};