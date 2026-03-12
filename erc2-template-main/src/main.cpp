#include <FEH.h>
#include <Arduino.h>

// Motor and encoder declarations
FEHMotor leftMotor(FEHMotor::Motor0, 6.0);
FEHMotor rightMotor(FEHMotor::Motor1, 6.0);
DigitalEncoder leftEncoder(FEHIO::Pin8);
DigitalEncoder rightEncoder(FEHIO::Pin10);

// Sensor declarations
AnalogInputPin optosensorLeft(FEHIO::Pin0);
AnalogInputPin optosensorMiddle(FEHIO::Pin2);
AnalogInputPin optosensorRight(FEHIO::Pin4);
AnalogInputPin cdsCell(FEHIO::Pin6);

// Variables
int intersectionCount = 0; // keep track of how many intersections we've gone through

// Constants
const float WHEEL_DIAMETER = 3.0;                                                          // in inches
const float ENCODER_COUNTS_PER_REVOLUTION = 318;                                           // ticks per revolution of the wheel
const float INCHES_PER_COUNT = (WHEEL_DIAMETER * 3.14159) / ENCODER_COUNTS_PER_REVOLUTION; // inches traveled per encoder tick
const float ROBOT_WIDTH = 7.15;                                                            // distance between the centers of the two wheels in inches
// const float ROBOT_LENGTH; //distance from the center of the robot to the front in inches
const float LEFT_OPTOSENSOR_THRESHOLD = 3;     // threshold value for left optosensor on the line (black line will have a value above this threshold, white background will have a value below this threshold)
const float MIDDLE_OPTOSENSOR_THRESHOLD = 4.6; // threshold value for middle optosensor on the line (black line will have a value above this threshold, white background will have a value below this threshold)
const float RIGHT_OPTOSENSOR_THRESHOLD = 3;    // threshold value for right optosensor on the line (black line will have a value above this threshold, white background will have a value below this threshold)

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
    while ((leftEncoder.Counts() + rightEncoder.Counts()) / 2 < targetCounts)
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

// This function will follow the line until it hits a branch
void followLineToIntersection(int percent)
{
    float lostLineTimer=0.0;
    bool isLost=false;

    /*
    State 0: 000, off the line (all sensors below threshold)
    State 1: 001, veering left (left and middle sensors are above threshold)
    State 4: 100, veering right (right and middle sensors are above threshold)
    State 2: 010, on the line (all three sensors are above threshold)
    State 7: 111, at an intersection (all three sensors are above threshold)
    State 3: 011, at an T intersection (middle and right sensors are above threshold)
    */

    // combine the three sensor readings into a 3 bit integer representing the current state

    while (1)
    { // keep following the line until hit an intersection
        float leftValue = optosensorLeft.Value();
        float middleValue = optosensorMiddle.Value();
        float rightValue = optosensorRight.Value();
        int currentState = ((leftValue > LEFT_OPTOSENSOR_THRESHOLD ? 1 : 0) << 2) // returns 1 if on line, 0 if off line
                           | ((middleValue > MIDDLE_OPTOSENSOR_THRESHOLD ? 1 : 0) << 1) | ((rightValue > RIGHT_OPTOSENSOR_THRESHOLD ? 1 : 0) << 0);
        // Debugs
        LCD.Clear();
        LCD.WriteLine(leftValue);
        LCD.WriteLine(middleValue);
        LCD.WriteLine(rightValue);
        LCD.WriteLine(currentState);

        if(currentState==7||currentState==3){
            leftMotor.Stop();
            rightMotor.Stop();
            break;
        }

        switch (currentState)
        {
        case 0:
            if(!isLost){
                lostLineTimer=TimeNow();
                isLost=true;
            }
            else if(TimeNow()-lostLineTimer>1){ // if it's been off the line for more than 0.5 seconds, stop and break out of the loop
                leftMotor.Stop();
                rightMotor.Stop();
                break;
            }
            break;
        case 1: // veering left, left and middle sensors are above threshold, turn right, left motor goes forward, right motor goes backward
            leftMotor.SetPercent(-percent);
            rightMotor.SetPercent(-percent);
            break;
        case 4: // veering right, right and middle sensors are above threshold, turn left, left motor goes backward, right motor goes forward
            leftMotor.SetPercent(percent);
            rightMotor.SetPercent(percent);
            break;
        case 2: // on the line, all three sensors are above threshold, go straight
            leftMotor.SetPercent(-percent);
            rightMotor.SetPercent(percent);
            break;
        }
        Sleep(0.05);
        
    }
}

void ERCMain()
{
    // Milestone 2
    // Step 1: Wait for the light, go backward and push the button, then orient towards the ramp, drive to the light at humidifier
    //  while(cdsCell.Value() < 3){} //wait for the light to turn on
    //  goForward(-50, 6); //go backward for 6 inches so that it hits the button
    //  goForward(50, 3); //go forward for 3 inches to get off the button
    //  turnRight(25,(45+18.7)); //45: from the tip of letter A to the corner facing the ramp, 18.7: from the corner facing the ramp to the center of the ramp
    //  goForward(50,7.41);
    //  turnLeft(25,(45+18.7)); //orient to front
    //  goForward(50,12.5); //to the beginning of the line on the upper level
    //  linefollowLineToIntersection(25); //follow the line until the first intersection, which is a T intersection with branch at thhe right
    //  goForward(50, 1); //go forward a little bit to get off the intersection
    //  linefollowLineToIntersection(25);
    //  goForward(50, 3); //go forward a little bit to get light under cds cell
    // while(1){
    //     LCD.Clear();
    //     LCD.WriteLine("left encoder counts: " + String(leftEncoder.Counts()));
    //     LCD.WriteLine("right encoder counts: " + String(rightEncoder.Counts()));
    //     goForward(25,6);
    //     turnLeft(25,90);
    //     goForward(-25,6);
    //     turnRight(25,90);
    // }
    // LCD.WriteLine(optosensorLeft.Value());
    // LCD.WriteLine(optosensorMiddle.Value());
    // LCD.WriteLine(optosensorRight.Value());
    // LCD.WriteLine(cdsCell.Value());
    // TestGUI();
    followLineToIntersection(25);
}
