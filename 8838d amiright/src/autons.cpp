#include "autons.h"

void redLeft()
{
    driveAngleOld(12, 0, 800, 10, 6);
}

void redRight()
{
}

void blueLeft()
{
}

void blueRight()
{
}

autonSelect currentAuton = redLeftAuton;

void nextAuton()
{
    switch (currentAuton)
    {
    case redLeftAuton:
        currentAuton = redRightAuton;
        break;
    case redRightAuton:
        currentAuton = blueLeftAuton;
        break;
    case blueLeftAuton:
        currentAuton = blueRightAuton;
        break;
    case blueRightAuton:
        currentAuton = redLeftAuton;
        break;
    default:
        currentAuton = redLeftAuton;
        break;
    }
}

void lastAuton()
{
    switch (currentAuton)
    {
    case redLeftAuton:
        currentAuton = blueRightAuton;
        break;
    case blueRightAuton:
        currentAuton = blueLeftAuton;
        break;
    case blueLeftAuton:
        currentAuton = redRightAuton;
        break;
    case redRightAuton:
        currentAuton = redLeftAuton;
        break;
    default:
        currentAuton = redLeftAuton;
        break;
    }
}

void runAuton()
{
    switch (currentAuton)
    {
    case redLeftAuton:
        redLeft();
        break;
    case redRightAuton:
        redRight();
        break;
    case blueLeftAuton:
        blueLeft();
        break;
    case blueRightAuton:
        blueRight();
        break;
    default:
        // do nothing
        break;
    }
}

const char *getAutonName(autonSelect currentA)
{
    switch (currentA)
    {
    case redLeftAuton:
        return "Current: Red Left";
    case redRightAuton:
        return "Current: Red Right";
    case blueLeftAuton:
        return "Current: Blue Left";
    case blueRightAuton:
        return "Current: Blue Right";
    default:
        return "Current: Unknown";
    }
}
