#include <FEH.h>
#include <Arduino.h>
// githubtest

// Motor and encoder declarations
FEHMotor leftMotor(FEHMotor::Motor0, 6.0);
FEHMotor rightMotor(FEHMotor::Motor1, 6.0);
DigitalEncoder leftEncoder(FEHIO::Pin8);
DigitalEncoder rightEncoder(FEHIO::Pin10);
FEHServo humidifierServo(FEHServo::Servo0);

// Sensor declarations
AnalogInputPin optosensorLeft(FEHIO::Pin0);
AnalogInputPin optosensorMiddle(FEHIO::Pin2);
AnalogInputPin optosensorRight(FEHIO::Pin4);
AnalogInputPin cdsCell(FEHIO::Pin6);

// Variables
int intersectionCount = 0;    // keep track of how many intersections we've gone through
bool humidifierIsRed = false; // keep track of whether the humidifier light is red or blue, initialized to false (blue) but will be updated when we see the light at the humidifier

// Constants
const float WHEEL_DIAMETER = 3.0;                                                          // in inches
const float ENCODER_COUNTS_PER_REVOLUTION = 318;                                           // ticks per revolution of the wheel
const float INCHES_PER_COUNT = (WHEEL_DIAMETER * 3.14159) / ENCODER_COUNTS_PER_REVOLUTION; // inches traveled per encoder tick
const float ROBOT_WIDTH = 7.15;
const float SERVO_BANDWIDTH = 20;
const float SECONDS_PER_DEGREE_NEG = 0.0023;  // 0.00235; //at 50% speed                                         // distance between the centers of the two wheels in inches
const float SECONDS_PER_DEGREE_POS = 0.00175; // 0.00235; //at 50% speed                                         // distance between the centers of the two wheels in inches
// const float ROBOT_LENGTH; //distance from the center of the robot to the front in inches
const float LEFT_OPTOSENSOR_THRESHOLD = 4;     // threshold value for left optosensor on the line (black line will have a value above this threshold, white background will have a value below this threshold)
const float MIDDLE_OPTOSENSOR_THRESHOLD = 4.6; // threshold value for middle optosensor on the line (black line will have a value above this threshold, white background will have a value below this threshold)
const float RIGHT_OPTOSENSOR_THRESHOLD = 4;    // threshold value for right optosensor on the line (black line will have a value above this threshold, white background will have a value below this threshold)
const float CDS_CELL_RED_THRESHOLD = 0.48;      // threshold value for cds cell to determine if the humidifier light is red or blue (red light will have a value below this threshold, blue light will have a value above this threshold)

// enums
//  enum INTERSECTION_TYPE{
//      FRONT_T_INTERSECTION,
//      RIGHT_T_INTERSECTION
//  }
//  enum ACTION_AT_INTERSECTION{
//      GO_STRAIGHT,
//      TURN_LEFT,
//      TURN_RIGHT
//  }

// Function declarations
void setServoSpeed(int percent)
{
    // int degree = 90+(percent*SERVO_BANDWIDTH/100);
    int degree = 90 + (percent * 90 / 100);
    humidifierServo.SetDegree(degree);
}
void turnServoByAngle(float angle)
{
    if (angle == 0)
        return;
    int operatingSpeed = 50;
    float timeToWait = 0.0;
    if (angle < 0)
    {
        operatingSpeed = -50;
        timeToWait = -angle * SECONDS_PER_DEGREE_NEG;
    }
    else
    {
        operatingSpeed = 50;
        timeToWait = angle * SECONDS_PER_DEGREE_POS;
    }
    setServoSpeed(operatingSpeed);
    Sleep(timeToWait);
    setServoSpeed(0);
}

// Left motor percentage is set to be negative since the motors are mounted in opposite directions
void goForward(int percent, float distance)
{
    // Calculate the number of encoder counts needed to travel the specified distance
    int targetCounts = distance / INCHES_PER_COUNT;

    // Reset encoders
    leftEncoder.ResetCounts();
    rightEncoder.ResetCounts();

    // Set motors to desired percent
    leftMotor.SetPercent(-percent);
    rightMotor.SetPercent(percent);

    // Keeps moving forward until the average of the two encoders reaches the target counts
    while ((abs(leftEncoder.Counts()) + abs(rightEncoder.Counts())) / 2 < targetCounts)
    {
    }

    // Stop motors
    leftMotor.Stop();
    rightMotor.Stop();
}
void upRamp(int percent, float distance)
{
    // Calculate the number of encoder counts needed to travel the specified distance
    int targetCounts = distance / INCHES_PER_COUNT;

    // Reset encoders
    leftEncoder.ResetCounts();
    rightEncoder.ResetCounts();

    // Set motors to desired percent
    leftMotor.SetPercent(-percent);
    rightMotor.SetPercent(percent + 7);

    // Keeps moving forward until the average of the two encoders reaches the target counts
    while ((abs(leftEncoder.Counts()) + abs(rightEncoder.Counts())) / 2 < targetCounts)
    {
    }

    // Stop motors
    leftMotor.Stop();
    rightMotor.Stop();
}
void turnLeft(int percent, float angle) // right motor goes forward, left motor goes backward
{
    // Calculate the number of encoder counts needed to turn the specified angle
    int targetCounts = (angle / 360.0) * (3.14159 * ROBOT_WIDTH / INCHES_PER_COUNT); // counts = fraction of a full turn * counts per full turn

    // Reset encoders
    leftEncoder.ResetCounts();
    rightEncoder.ResetCounts();

    // Set motors to desired percent (left motor goes backward, right motor goes forward)
    leftMotor.SetPercent(percent);
    rightMotor.SetPercent(percent);

    // Keeps turning until the average of the two encoders reaches the target counts
    while ((abs(leftEncoder.Counts()) + abs(rightEncoder.Counts())) / 2 < targetCounts)
    {
    }

    // Stop motors
    leftMotor.Stop();
    rightMotor.Stop();
}
void turnRight(int percent, float angle) // left motor goes forward, right motor goes backward
{
    // Calculate the number of encoder counts needed to turn the specified angle
    int targetCounts = (angle / 360.0) * (3.14159 * ROBOT_WIDTH / INCHES_PER_COUNT); // counts = fraction of a full turn * counts per full turn

    // Reset encoders
    leftEncoder.ResetCounts();
    rightEncoder.ResetCounts();

    // Set motors to desired percent (left motor goes forward, right motor goes backward)
    leftMotor.SetPercent(-percent);
    rightMotor.SetPercent(-percent);

    // Keeps turning until the average of the two encoders reaches the target counts
    while ((abs(leftEncoder.Counts()) + abs(rightEncoder.Counts())) / 2 < targetCounts)
    {
    }

    // Stop motors
    leftMotor.Stop();
    rightMotor.Stop();
}

// Runtime thresholds, can be updated by calibration so local-only center line testing
// and final field (black-white-black) can share one algorithm.
float leftLineThreshold = LEFT_OPTOSENSOR_THRESHOLD;
float middleLineThreshold = MIDDLE_OPTOSENSOR_THRESHOLD;
float rightLineThreshold = RIGHT_OPTOSENSOR_THRESHOLD;

// Automatically estimate line thresholds from current floor + black line.
// Assumption: during calibration the robot is placed with middle sensor on black line.
void calibrateLineThresholds(float safetyMargin)
{
    float leftFloor = optosensorLeft.Value();
    float middleLine = optosensorMiddle.Value();
    float rightFloor = optosensorRight.Value();

    leftLineThreshold = (leftFloor + middleLine) * 0.5 + safetyMargin;
    middleLineThreshold = middleLine - safetyMargin;
    rightLineThreshold = (rightFloor + middleLine) * 0.5 + safetyMargin;

    LCD.Clear();
    LCD.WriteLine("Line threshold calibrated");
    LCD.WriteLine(leftLineThreshold);
    LCD.WriteLine(middleLineThreshold);
    LCD.WriteLine(rightLineThreshold);
    Sleep(0.8);
}

int readLineState()
{
    float leftValue = optosensorLeft.Value();
    float middleValue = optosensorMiddle.Value();
    float rightValue = optosensorRight.Value();

    return ((leftValue > leftLineThreshold ? 1 : 0) << 2)
           | ((middleValue > middleLineThreshold ? 1 : 0) << 1)
           | ((rightValue > rightLineThreshold ? 1 : 0) << 0);
}

// One-wheel steering mode (no PID, no differential speed).
// left motor sign is negative when moving robot forward.
void driveStraightSimple(int percent)
{
    leftMotor.SetPercent(-percent);
    rightMotor.SetPercent(percent);
}

void pivotLeftSimple(int percent)
{
    leftMotor.SetPercent(0);
    rightMotor.SetPercent(percent);
}

void pivotRightSimple(int percent)
{
    leftMotor.SetPercent(-percent);
    rightMotor.SetPercent(0);
}

// This function follows the line and supports both single-black-line practice field
// and final field black-white-black pattern by using an adaptive state machine.
void followLineToIntersection(int percent)
{
    const float LOST_LINE_TIMEOUT = 0.9;
    const float INTERSECTION_HOLD_TIME = 0.12;

    float lostLineTimer = 0.0;
    float allOnTimer = 0.0;
    bool isLost = false;
    bool allOn = false;
    int lastTurnDirection = 1; // 1 means last correction was right, -1 means left

    while (1)
    {
        int currentState = readLineState();

        // 111 usually means branch/intersection in final field. Require persistence to
        // avoid false trigger when passing over edge noise.
        if (currentState == 7)
        {
            if (!allOn)
            {
                allOn = true;
                allOnTimer = TimeNow();
            }
            else if (TimeNow() - allOnTimer > INTERSECTION_HOLD_TIME)
            {
                leftMotor.Stop();
                rightMotor.Stop();
                return;
            }
        }
        else
        {
            allOn = false;
        }

        switch (currentState)
        {
        case 2: // 010 centered
            isLost = false;
            driveStraightSimple(percent);
            break;

        case 6: // 110 line appears on left + middle, robot drifted right -> steer left
        case 4: // 100 far right drift
            isLost = false;
            lastTurnDirection = -1;
            pivotLeftSimple(percent);
            break;

        case 3: // 011 line appears on right + middle, robot drifted left -> steer right
        case 1: // 001 far left drift
            isLost = false;
            lastTurnDirection = 1;
            pivotRightSimple(percent);
            break;

        case 5: // 101 usually black-white-black border in final field
            // Treat as centered region and keep going instead of mis-detecting intersection.
            isLost = false;
            driveStraightSimple(percent);
            break;

        case 0: // 000 lost the line completely
        default:
            if (!isLost)
            {
                lostLineTimer = TimeNow();
                isLost = true;
            }

            if (TimeNow() - lostLineTimer > LOST_LINE_TIMEOUT)
            {
                leftMotor.Stop();
                rightMotor.Stop();
                return;
            }

            // keep searching by continuing the last correction direction
            if (lastTurnDirection > 0)
            {
                pivotRightSimple(percent);
            }
            else
            {
                pivotLeftSimple(percent);
            }
            break;
        }

        Sleep(0.03);
    }
}

void hitButton(int percent, int angle)
{
    if (cdsCell.Value() < CDS_CELL_RED_THRESHOLD)
    {
        humidifierIsRed = true;
        LCD.WriteLine("humidifier is red");
        goForward(25, 3.93); // reach humidifier
        // Calculate the number of encoder counts needed to turn the specified angle
        int targetCounts = (angle / 360.0) * (3.14159 * ROBOT_WIDTH / INCHES_PER_COUNT); // counts = fraction of a full turn * counts per full turn

        // Reset encoders
        leftEncoder.ResetCounts();
        rightEncoder.ResetCounts();

        // Set motors to desired percent (left motor goes backward, right motor goes forward)
        leftMotor.SetPercent(-percent);
        rightMotor.SetPercent(0);

        // Keeps turning until the average of the two encoders reaches the target counts
        while ((abs(leftEncoder.Counts()) + abs(rightEncoder.Counts())) / 2 < targetCounts)
        {
        }

        // Stop motors
        leftMotor.Stop();
        rightMotor.Stop();
    }
    else
    {
        LCD.WriteLine("humidifier is blue");
        goForward(25, 4); // reach humidifier
        // Calculate the number of encoder counts needed to turn the specified angle
        int targetCounts = (angle / 360.0) * (3.14159 * ROBOT_WIDTH / INCHES_PER_COUNT); // counts = fraction of a full turn * counts per full turn

        // Reset encoders
        leftEncoder.ResetCounts();
        rightEncoder.ResetCounts();

        // Set motors to desired percent (left motor goes backward, right motor goes forward)
        leftMotor.SetPercent(-percent);
        rightMotor.SetPercent(0);

        // Keeps turning until the average of the two encoders reaches the target counts
        while ((abs(leftEncoder.Counts()) + abs(rightEncoder.Counts())) / 2 < targetCounts)
        {
        }

        // Stop motors
        leftMotor.Stop();
        rightMotor.Stop();
    }
}

void ERCMain()
{

    // TestGUI();
    // negative pushing left button, positive pushing right button
    // initialize servo
    //  humidifierServo.SetMin(1024);
    //  humidifierServo.SetMax(1876);
    //  LCD.WriteLine("servo calibrated");
    //  turnServoByAngle(-40);
    //  Sleep(2.0);
    //  LCD.WriteLine("feedback for initial twist");
    //  Sleep(2.0);
    //  turnServoByAngle(-50);
    //  LCD.WriteLine("turned servo by -20 degrees");
    //  Sleep(2.0);
    //  turnServoByAngle(50);
    //  LCD.WriteLine("turned servo by 20 degrees");


    calibrateLineThresholds(0.12);

    // Milestone 2
    // Step 1: Wait for the light, go backward and push the button, then orient towards the ramp, drive to the light at humidifier
    while (cdsCell.Value() > 1)
    {
    } // wait for the light to turn on
    LCD.WriteLine("light detected, starting!");
    goForward(-75, 3); // go backward for 3 inches so that it hits the button
    LCD.WriteLine("button pushed");
    goForward(50, 1); // go forward for 1 inch to get off the button
    LCD.WriteLine("getting off button");
    turnRight(25, (45 + 10)); // 45: from the tip of letter A to the corner facing the ramp, 18.7: from the corner facing the ramp to the center of the ramp
    LCD.WriteLine("oriented towards ramp");
    goForward(50, 16);
    LCD.WriteLine("reached the beginning of the ramp");
    //  turnLeft(25,(18.7)); //orient to front
    //  LCD.WriteLine("oriented forward");
    upRamp(50, 13); // to the beginning of the line on the upper level
    LCD.WriteLine("on the line on upper level");
    goForward(50, 11.7);
    LCD.WriteLine("at the corner");
    turnLeft(25, 100); // turn left at the T intersection to face the ramp
    LCD.WriteLine("turned left at the corner");
    goForward(50, 11.5);
    LCD.WriteLine("reached before the light");
    int pulsePower = 20;
    float pulseDuration = 0.5;
    LCD.WriteLine(cdsCell.Value());
    while (cdsCell.Value() > 1)
    { // keep pulsing forward until the light is red
        LCD.WriteLine("pulsing forward");
        leftMotor.SetPercent(-pulsePower);
        rightMotor.SetPercent(pulsePower);
        Sleep(pulseDuration);
        leftMotor.Stop();
        rightMotor.Stop();
        Sleep(pulseDuration);
        LCD.WriteLine(cdsCell.Value());
    }
    int degree = 30;
    LCD.WriteLine("reached LED");
    LCD.WriteLine(cdsCell.Value());
    hitButton(100,90);

    Sleep(2.0);

    // Bonus: get back to the starting point
    // LCD.Clear();
    // turnLeft(25, 200);
    // LCD.WriteLine("turned around");
    // goForward(50, 18);
    // LCD.WriteLine("reached corner");
    // turnRight(25, 90);
    // LCD.WriteLine("oriented towards ramp");
    // goForward(50, 11.7);
    // LCD.WriteLine("reached the lower level");
    // goForward(50, 16);
    // LCD.WriteLine("reached the letter");
    // turnRight(25, 45 + 18.7);
    // LCD.WriteLine("oriented towards button");
    // goForward(50, 2);
    // LCD.WriteLine("pushed button");
}
// min at full speed 884
// max at full speed 1876
// stop point 1450
// clkwise width: 1450-1380=70
// counterclkwise width: 1526-1450=76