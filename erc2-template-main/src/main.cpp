#include <FEH.h>
#include <FEHServo.h>
#include <Arduino.h>
// githubtest

// Motor and encoder declarations
FEHMotor leftMotor(FEHMotor::Motor2, 6.0);
FEHMotor rightMotor(FEHMotor::Motor0, 6.0);
DigitalEncoder leftEncoder(FEHIO::Pin10);
DigitalEncoder rightEncoder(FEHIO::Pin8);
FEHServo humidifierServo(FEHServo::Servo0);
FEHServo sideArmServo(FEHServo::Servo1);
FEHServo compostServo(FEHServo::Servo3);

// Sensor declarations
AnalogInputPin optosensorLeft(FEHIO::Pin4);
AnalogInputPin optosensorMiddle(FEHIO::Pin2);
AnalogInputPin optosensorRight(FEHIO::Pin0);
AnalogInputPin cdsCell(FEHIO::Pin6);

// Variables
int intersectionCount = 0;    // keep track of how many intersections we've gone through
bool humidifierIsRed = false; // keep track of whether the humidifier light is red or blue, initialized to false (blue) but will be updated when we see the light at the humidifier

// Constants
const float WHEEL_DIAMETER = 3.0;                                                          // in inches
const float ENCODER_COUNTS_PER_REVOLUTION = 318;                                           // ticks per revolution of the wheel
const float INCHES_PER_COUNT = (WHEEL_DIAMETER * 3.14159) / ENCODER_COUNTS_PER_REVOLUTION; // inches traveled per encoder tick
const float ROBOT_WIDTH = 5.748;                                                           // in inches
const float LEFT_OPTOSENSOR_THRESHOLD = 4;                                                 // threshold value for left optosensor on the line (black line will have a value above this threshold, white background will have a value below this threshold)
const float MIDDLE_OPTOSENSOR_THRESHOLD = 4;                                               // threshold value for middle optosensor on the line (black line will have a value above this threshold, white background will have a value below this threshold)
const float RIGHT_OPTOSENSOR_THRESHOLD = 4;                                                // threshold value for right optosensor on the line (black line will have a value above this threshold, white background will have a value below this threshold)
const float CDS_CELL_RED_THRESHOLD = 0.48;                                                 // threshold value for cds cell to determine if the humidifier light is red or blue (red light will have a value below this threshold, blue light will have a value above this threshold)
const float RCS_WAIT_TIME_IN_SEC = 0.35;                                                   // wait time between RCS position requests
const float PULSE_TIME = 0.2;                                                              // duration of each pulse for RCS adjustments
const int PULSE_POWER = 25;                                                                // power of each pulse for RCS adjustments
const int MINUS = 0;                                                                       // constant for AruCo code orientation, means the robot needs to pulse counterclockwise to adjust
const int PLUS = 1;                                                                        // constant for AruCo code orientation, means the robot needs to pulse clockwise to adjust

// Function declarations
// 1. Movement functions
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

void pivotLeft(int percent, float angle) // right motor goes forward, left motor stops
{
    // Calculate the number of encoder counts needed to turn the specified angle
    int targetCounts = (angle / 360.0) * (3.14159 * ROBOT_WIDTH * 2 / INCHES_PER_COUNT); // counts = fraction of a full turn * counts per full turn

    // Reset encoders
    leftEncoder.ResetCounts();
    rightEncoder.ResetCounts();

    // Set motors to desired percent (left motor goes backward, right motor goes forward)
    leftMotor.SetPercent(0);
    rightMotor.SetPercent(percent);

    // Keeps turning until the average of the two encoders reaches the target counts
    while ((abs(leftEncoder.Counts()) + abs(rightEncoder.Counts())) / 2 < targetCounts)
    {
    }

    // Stop motors
    leftMotor.Stop();
    rightMotor.Stop();
}
void pivotRight(int percent, float angle)
{
    // Calculate the number of encoder counts needed to turn the specified angle
    int targetCounts = (angle / 360.0) * (3.14159 * ROBOT_WIDTH * 2 / INCHES_PER_COUNT); // counts = fraction of a full turn * counts per full turn

    // Reset encoders
    leftEncoder.ResetCounts();
    rightEncoder.ResetCounts();

    // Set motors to desired percent (left motor goes forward, right motor goes backward)

    leftMotor.SetPercent(percent);
    rightMotor.SetPercent(0);
    while ((abs(leftEncoder.Counts()) + abs(rightEncoder.Counts())) / 2 < targetCounts)
    {
    }

    // Stop motors
    leftMotor.Stop();
    rightMotor.Stop();
}

// 2. Line following functions
//  Runtime thresholds, can be updated by calibration so local-only center line testing
//  and final field (black-white-black) can share one algorithm.
float leftLineThreshold = LEFT_OPTOSENSOR_THRESHOLD;
float middleLineThreshold = MIDDLE_OPTOSENSOR_THRESHOLD;
float rightLineThreshold = RIGHT_OPTOSENSOR_THRESHOLD;
int readLineState()
{
    float leftValue = optosensorLeft.Value();
    float middleValue = optosensorMiddle.Value();
    float rightValue = optosensorRight.Value();

    return ((leftValue > leftLineThreshold ? 1 : 0) << 2) | ((middleValue > middleLineThreshold ? 1 : 0) << 1) | ((rightValue > rightLineThreshold ? 1 : 0) << 0);
}
/*
State 0: 000, all sensors not on line, drive a little bit to see if that's a course issue, then stop
state 1: 001, right sensor only on line, veering left, need to correct right, left motor faster
state 2: 010, middle sensor only on line, go straight
state 3: 011, middle + right sensor on line, veering left or at a right branch
state 4: 100, left sensor only on line, veering right, need to correct left, right motor faster
state 5: 101, left + right sensor on line, not likely to happen, if that's the case something must went wrong
state 6: 110, left + middle sensor on line, veering right or at a left branch
state 7: 111, all sensors on line, probably at an intersection, stop
*/
void followLinePID(float base_percent)
{
    const float LOST_LINE_TIMEOUT = 0.9;
    const float INTERSECTION_HOLD_TIME = 0.2;

    float lostLineTimer = 0.0;
    float allOnTimer = 0.0;
    bool isLost = false;
    bool allOn = false;

    // PID constants
    const float Kp = 15;   // Kp, base speed variance for every error, change this first if the robot is not responsive enough or too responsive
    const float Ki = 0.03; // Ki, integral term: corrects long-term deviation (e.g., motor imbalance)
    const float Kd = 0.7;  // Kd, derivative term: predicts trend, resists inertia during turns, reduces left-right "drawing"

    float error = 0.0;
    float last_error = 0.0;
    float integral = 0.0;
    float derivative = 0.0;

    const float MAX_SPEED = base_percent;       // base speed for straight line
    const float MID_SPEED = base_percent * 0.7; // base speed for inner wheels when turning
    const float MIN_SPEED = base_percent * 0.3; // base speed for inner wheels when sharp turning

    while (1)
    {
        LCD.Clear();
        LCD.WriteLine("Error:");
        LCD.WriteLine(error);
        LCD.WriteLine("current state:");
        LCD.WriteLine(readLineState());
        LCD.WriteLine("Left sensor:");
        LCD.WriteLine(optosensorLeft.Value());
        LCD.WriteLine("Middle sensor:");
        LCD.WriteLine(optosensorMiddle.Value());
        LCD.WriteLine("Right sensor:");
        LCD.WriteLine(optosensorRight.Value());

        int currentState = readLineState();

        // 1. cross road: ---
        //                 |
        //
        if (currentState == 7)
        {
            if (!allOn)
            {
                allOn = true;
                allOnTimer = TimeNow();
            }
            else if (TimeNow() - allOnTimer > INTERSECTION_HOLD_TIME)
            {
                LCD.WriteLine("Intersection!");
                LCD.WriteLine(TimeNow() - allOnTimer);
                leftMotor.Stop();
                rightMotor.Stop();
                return;
            }
        }
        else
        {
            allOn = false;
        }

        // making current position to error value for PID to process
        switch (currentState)
        {
        case 2: // 010: centered
            error = 0;
            isLost = false;
            break;
        case 6: // 110: veer right, need to turn left
            error = 1;
            isLost = false;
            break;
        case 4: // 100: veering right a lot, need to turn left hard
            error = 2;
            isLost = false;
            break;
        case 3: // 011: veering left a bit, need to turn right
            error = -1;
            isLost = false;
            break;
        case 1: // 001: veering left a lot, need to turn right hard
            error = -2;
            isLost = false;
            break;
        case 5: // 101: not likely, but if happens might be at a border of line, treat as centered
            error = 0;
            isLost = false;
            break;
        case 7:
            isLost = false;
            break;
        case 0: // 000: lost line
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
                LCD.WriteLine("Line lost, stopping.");
                return;
            }
            break;
        }

        float current_base_speed = 0;
        if (error == 0)
        {
            current_base_speed = MAX_SPEED;
        }
        else if (abs(error) == 1)
        {
            current_base_speed = MID_SPEED;
        }
        else
        {
            current_base_speed = MIN_SPEED;
        }

        // PID calculations
        if (error == 0)
        {
            integral = 0;
        }
        else
        {
            integral = integral + error;
        }

        derivative = error - last_error;

        // overall adjustments based on PID terms
        float adjustment = (Kp * error) + (Ki * integral) + (Kd * derivative);

        // making adjustment to motor power
        // if error is positive, means we need to turn left, so left motor should be slower than right motor
        float left_power = current_base_speed - adjustment;
        float right_power = current_base_speed + adjustment;
        LCD.WriteLine("Left power:");
        LCD.WriteLine(left_power);
        LCD.WriteLine("Right power:");
        LCD.WriteLine(right_power);

        // limiting power to be between 0 and 100
        if (left_power > 100.0)
            left_power = 100.0;
        if (left_power < 0.0)
            left_power = 0.0;
        if (right_power > 100.0)
            right_power = 100.0;
        if (right_power < 0.0)
            right_power = 0.0;

        leftMotor.SetPercent(-left_power);
        rightMotor.SetPercent(right_power);

        last_error = error;

        Sleep(0.01);
    }
}

// 3. RCS related functions
void pulseCounterclockwise(int percent, float duration) // left motor goes backward, right motor goes forward
{
    leftMotor.SetPercent(percent);
    rightMotor.SetPercent(percent);
    Sleep(duration);
    leftMotor.Stop();
    rightMotor.Stop();
}

void pulseClockwise(int percent, float duration) // left motor goes forward, right motor goes backward
{
    leftMotor.SetPercent(-percent);
    rightMotor.SetPercent(-percent);
    Sleep(duration);
    leftMotor.Stop();
    rightMotor.Stop();
}

void pulseForward(int percent, float duration) // both motors go forward
{
    leftMotor.SetPercent(-percent);
    rightMotor.SetPercent(percent);
    Sleep(duration);
    leftMotor.Stop();
    rightMotor.Stop();
}

void getRCSLocation()
{
    RCSPose *pose = RCS.RequestPosition();
    LCD.SetBackgroundColor(LCD.Black);
    LCD.SetFontColor(LCD.White);
    while (pose == nullptr)
    {
        LCD.WriteLine("Waiting for RCS position...");
        Sleep(0.1);
        pose = RCS.RequestPosition();
        LCD.Clear();
    }
    LCD.Clear();
    LCD.WriteLine("RCS Position:");
    LCD.WriteLine("X:");
    LCD.WriteLine(pose->x);
    LCD.WriteLine("Y:");
    LCD.WriteLine(pose->y);
    LCD.WriteLine("Heading:");
    LCD.WriteLine(pose->heading);
}

void check_position(float target_coord, bool is_x_axis, float target_heading, int orientation)
{
    RCSPose *pose;
    int base_power = (orientation == MINUS) ? -PULSE_POWER : PULSE_POWER;
    int max_attempts = 3;

    for (int attempt = 0; attempt < max_attempts; attempt++)
    {
        pose = RCS.RequestPosition();
        while (pose == nullptr || pose->heading < 0)
        {
            Sleep(0.1);
            pose = RCS.RequestPosition();
        }

        float current_coord = is_x_axis ? pose->x : pose->y;
        float coord_error = current_coord - target_coord;

        float heading_error = target_heading - pose->heading;
        if (heading_error > 180.0)
            heading_error -= 360.0;
        else if (heading_error < -180.0)
            heading_error += 360.0;

        if (abs(coord_error) <= 1.0 && abs(heading_error) <= 1.0)
        {
            break;
        }

        if (current_coord > target_coord + 1.0)
        {
            goForward(-base_power, abs(coord_error));
            Sleep(RCS_WAIT_TIME_IN_SEC);
        }
        else if (current_coord < target_coord - 1.0)
        {
            goForward(base_power, abs(coord_error));
            Sleep(RCS_WAIT_TIME_IN_SEC);
        }

        pose = RCS.RequestPosition();
        while (pose == nullptr || pose->heading < 0)
        {
            Sleep(0.1);
            pose = RCS.RequestPosition();
        }

        heading_error = target_heading - pose->heading;
        if (heading_error > 180.0)
            heading_error -= 360.0;
        else if (heading_error < -180.0)
            heading_error += 360.0;

        if (abs(heading_error) > 1.0)
        {
            if (heading_error > 0)
            {
                turnLeft(PULSE_POWER, abs(heading_error));
            }
            else
            {
                turnLeft(-PULSE_POWER, abs(heading_error));
            }
            Sleep(RCS_WAIT_TIME_IN_SEC);
        }
    }
}

// 5. Task movement functions


void openWindow()
{
// drive forward
    goForward(30, 10.0); 

// turn left degrees to face window
    turnRight(30, 90.0);

// drive forward to push window
    goForward(30, 8.0); 

// go around the window
    // Back up to clear the window frame 
    goForward(-30, 4.0); 

// Turn right 45 degrees to be at an angle with window
    turnRight(30, 45.0);

// drive forward to pass the open window
    goForward(30, 2.0);

// turn left to be at an angle head on with the window
    turnLeft(30, 90.0);

// drive forward to clear the back edge of the window
    goForward(30, 2.0);

// Turn left 45 degrees to be parallel with the window
    turnRight(30, 45.0);

// drive forward to clear the window
    goForward(30, 2.0); 

// drive backward to push the window back to its closed position
    goForward(-30, 10.0); 
    
// back up slightly after closing to smoothly move to next task
    goForward(-30, 3.0);
}


// 6. Task specific functions

// 7. Others
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
{ // initialize
    sideArmServo.SetMin(588);
    sideArmServo.SetMax(2120);
    RCS.InitializeTouchMenu("0800A5DYF");
    RCS.DisableRateLimit();
    RCSPose *pose;
    int touch_x, touch_y;
    while(!LCD.Touch(&touch_x, &touch_y))
    {
        getRCSLocation();
    }
    Sleep(2);
    // PLACE TASK FUNCTION CALLS HERE




    // TestGUI();
    // while (cdsCell.Value() > 1)
    // {
    // }
}



