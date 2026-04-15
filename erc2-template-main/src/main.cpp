#include <FEH.h>
#include <FEHServo.h>
#include <Arduino.h>
#include <math.h>
// githubtest

// Motor and encoder declarations
FEHMotor leftMotor(FEHMotor::Motor2, 6.0);
FEHMotor rightMotor(FEHMotor::Motor0, 6.0);
DigitalEncoder leftEncoder(FEHIO::Pin10);
DigitalEncoder rightEncoder(FEHIO::Pin8);
FEHServo humidifierServo(FEHServo::Servo0);
FEHServo sideArmServo(FEHServo::Servo1);
FEHServo compostServo(FEHServo::Servo3);
FEHServo bigArmServo(FEHMotor::Servo);

// Sensor declarations
AnalogInputPin optosensorLeft(FEHIO::Pin4);
AnalogInputPin optosensorMiddle(FEHIO::Pin2);
AnalogInputPin optosensorRight(FEHIO::Pin0);
AnalogInputPin cdsCell(FEHIO::Pin6);

// Variablesrcs
int intersectionCount = 0;    // keep track of how many intersections we've gone through
bool humidifierIsRed = false; // keep track of whether the humidifier light is red or blue, initialized to false (blue) but will be updated when we see the light at the humidifier
int RCSRequestCount = 0;      // keep track of how many times we've requested RCS position, to avoid infinite loop if RCS is not working properly

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

float distToPoint(float currentX, float currentY, float targetX, float targetY);
float angleToPoint(float currentX, float currentY, float targetX, float targetY);

// servo methods

void turnServoByAngle(float angle, FEHServo servo)
{
    servo.SetMax
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

void check_position(float target_x, float target_y, float target_heading)
{
    LCD.Clear();
    LCD.WriteLine("Target Pos: X, Y, Head");
    LCD.WriteLine(target_x);
    LCD.WriteLine(target_y);
    LCD.WriteLine(target_heading);
    RCSPose *pose;
    
    int max_attempts = 15; 

    for (int attempt = 0; attempt < max_attempts; attempt++)
    {
        pose = RCS.RequestPosition();
        RCSRequestCount++;
        while (pose == nullptr || pose->heading < 0)
        {
            Sleep(0.1);
            pose = RCS.RequestPosition();
            RCSRequestCount++;
        }

        float corrected_heading = pose->heading + 180.0;
        if (corrected_heading >= 360.0) corrected_heading -= 360.0;

        float dx = target_x - pose->x;
        float dy = target_y - pose->y;
        float distance = sqrt(dx * dx + dy * dy);

        float heading_error = target_heading - corrected_heading; 
        if (heading_error > 180.0) heading_error -= 360.0;
        else if (heading_error < -180.0) heading_error += 360.0;

        // 容差判断
        if (distance <= 1 && abs(heading_error) <= 2.0)
        {
            LCD.WriteLine("Position Reached!");
            break;
        }

        if (distance > 1)
        {
            float absolute_angle_to_target = atan2(dy, dx) * 180.0 / 3.14159265;
            if (absolute_angle_to_target < 0) absolute_angle_to_target += 360.0;

            float turn_to_target = absolute_angle_to_target - corrected_heading;
            if (turn_to_target > 180.0) turn_to_target -= 360.0;
            else if (turn_to_target < -180.0) turn_to_target += 360.0;

            int current_power = PULSE_POWER;

            if (turn_to_target > 90.0) 
            {
                turn_to_target -= 180.0;  
                current_power = -PULSE_POWER; 
            } 
            else if (turn_to_target < -90.0) 
            {
                turn_to_target += 180.0;  
                current_power = -PULSE_POWER; 
            }

            if (abs(turn_to_target) > 2.0)
            {
                if (turn_to_target > 0) turnLeft(PULSE_POWER, abs(turn_to_target));
                else turnRight(PULSE_POWER, abs(turn_to_target));
                Sleep(RCS_WAIT_TIME_IN_SEC);
            }

            pulseForward(current_power, PULSE_TIME); 
            Sleep(RCS_WAIT_TIME_IN_SEC);
        }
        else 
        {
            if (abs(heading_error) > 2.0)
            {
                if (heading_error > 0) turnLeft(PULSE_POWER, abs(heading_error));
                else turnRight(PULSE_POWER, abs(heading_error));
                Sleep(RCS_WAIT_TIME_IN_SEC);
            }
        }
        LCD.WriteLine("current position: X, Y, Head");
        LCD.WriteLine(pose->x);
        LCD.WriteLine(pose->y);
        LCD.WriteLine(pose->heading);
        Sleep(0.5);
        LCD.Clear();
    }
    LCD.Clear();
    LCD.WriteLine("Current Request count:");
    LCD.WriteLine(RCSRequestCount);
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
    int touch_x, touch_y;
    while (!LCD.Touch(&touch_x, &touch_y))
    {
        getRCSLocation();
    }
    Sleep(2);

    // PLACE TASK FUNCTION CALLS HERE
}

// assume arm servo starts at correct height
void pickUpFruit()
{
    // move to compost? where is compost function ending?

    // measure values - facing away from bucket
    check_position();

    goForward(40, 3); // 3 is place holder value
    turnServoByAngle(360, bigArmServo);
}
// proceed to back away and go up ramp and stuff

// assuming arm can reach table
void depositFruitTable()
{
    // drive forward to ramp, turn, and up the ramp
    goForward(50, 25);
    turnLeft(30, 90);
    goForward(80, 13);
    // continue to table
    goForward(30, 15);
    turnLeft(30, 90);
    goForward(30, 5);
    turnRight(30, 90);
    goForward(30, 5);
    turnLeft(30, 90);
    check_position();

    // needs testing adjust values
    turnServoByAngle(5 * 360, bigArmServo);
    turnServoByAngle(-360, bigArmServo);
}

// assuming crate
void depositFruitCrate()
{
    // drive forward to ramp, turn, and up the ramp
    goForward(50, 25);
    turnLeft(30, 90);
    goForward(80, 13);
    // continue to table
    goForward(30, 15);
    turnLeft(30, 90);
    goForward(30, 5);
    turnRight(30, 90);
    goForward(30, 5);
    // turn around
    turnRight(30, 180);
    goForward(-30, 4);
    check_position();

    // drop into crate
    turnServoByAngle(-3 * 360, bigArmServo);
    goForward(-30, 2);

    // turn to fertilizer
    turnRight(30, 90);
}

void navigateToFertilizer()
{
    // rcs bs
    goForward(30, 10);
    int state = RCS.GetLever();

    switch (state)
    {
    case 1:
        navigateToPoint(, , 315);
        check_position();
        break;
    case 2:
        navigateToPoint(, , 315);
        check_position();
        break;
    case 3:
        navigateToPoint(, , 315);
        check_position();
        break;
    }

    fertilizer();
}

void fertilizer()
{
    // raise arm
    turnServoByAngle(3 * 360, bigArmServo);
    goForward(-30, 3);
    turnServoByAngle(-5 * 360, bigArmServo);
    goForward(-30, 2);
    Sleep(5.5);
    goForward(30, 2);

    // raise arm
    turnServoByAngle(5 * 360, bigArmServo);
}

void goToHumidifer()
{
    // x and y of light?
    navigateToPoint(, , 315);
}

// distance from light to buttons is 9 in
// when does this start?
void humidifer()
{

    check_position();
    float cellValue = cdsCell.Value();
    // when the robot is on the light
    // update thresholds at some point idk what they are
    //  red
    if (cellValue > 1)
    {
        turnRight(30, 90);
        goForward(30, 2.25);
        turnLeft(30, 90);
        goForward(30, 7.35);
    }
    else if (cellValue < 1 && cellValue > 0)
    {
        turnLeft(30, 90);
        goForward(30, 2.25);
        turnRight(30, 90);
        goForward(30, 7.35);
    }
    else
    {
        LCD.WriteLine("CdScell not reading correctly");
    }
}

void navigateToCompost()
{
    LCD.WriteLine("getting off button");
    turnLeft(25, 90);
    LCD.WriteLine("turning towards compost");
    goForward(50, 2);
    LCD.WriteLine("drive towards the bin");
    turnRight(25, 45);
    LCD.WriteLine("turning towards compost");
    goForward(50, 1);
    LCD.WriteLine("get attached to bin");
    turnLeft(25, 10);
    LCD.WriteLine("turning towards compost");
    goForward(50, 1);
    LCD.WriteLine("get attached to bin");
    turnRight(25, 5);
    LCD.WriteLine("turning towards compost");
    goForward(50, 4);
    LCD.WriteLine("get attached to bin");
    compostTurn(-480);
    LCD.WriteLine("turn compost ccw");
    Sleep(2.0);
    LCD.WriteLine("return compost to intial");
    compostTurn(550);
    LCD.WriteLine("turning towards compost");
    goForward(-50, 5);
    LCD.WriteLine("reverse");
    turnLeft(25, 15);
    LCD.WriteLine("turn towards button");
    goForward(-50, 1);
    LCD.WriteLine("get to button");
    turnRight(25, 10);
    LCD.WriteLine("turn towards button");
    goForward(-60, 8);
    LCD.WriteLine("get to button");
}

void compostSpeed(int percent)
{
    // int degree = 90+(percent*SERVO_BANDWIDTH/100);
    int degree = 90 + (percent * 90 / 100);
    compostServo.SetDegree(degree);
}

void compostTurn(int angle)
{
    compostServo.SetMin(500);
    compostServo.SetMax(2500);

    if (angle == 0)
        return;
    int operatingSpeed = 90;
    float timeToWait = 1.0;
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
    compostSpeed(operatingSpeed);
    Sleep(timeToWait);

    compostServo.Off();
}

// some other navigatino stuff

float distToPoint(float currentX, float currentY, float targetX, float targetY)
{
    float xDist = targetX - currentX;
    float yDist = targetY - currentY;
    return pow((pow(xDist, 2) + pow(yDist, 2)), 0.5);
}

float angleToPoint(float currentX, float currentY, float targetX, float targetY)
{
    float xDist = targetX - currentX;
    float yDist = targetY - currentY;
    float tempAngle = atan(xDist / yDist);

    if (xDist > 0 && yDist > 0)
    {
        return tempAngle;
    }
    else if (xDist > 0 && yDist < 0)
    {
        return 360 - tempAngle;
    }
    else if (xDist < 0 && yDist > 0)
    {
        return 180 - tempAngle;
    }
    else
        (xDist < 0 && yDist < 0);
    {
        return 270 - tempAngle;
    }
}

void navigateToPoint(float targetX, float targetY, float targetAngle)
{
    RCSPose *pose = RCS.RequestPosition();

    float dist = distToPoint(pose->x, pose->y, targetX, targetY);
    float angle = angleToPoint(pose->x, pose->y, targetX, targetY);

    if (pose->heading > angle)
    {
        turnRight(30, pose->heading - angle);
    }
    else
    {
        turnLeft(30, angle - pose->heading);
    }

    goForward(30, dist);

    if (pose->heading > targetAngle)
    {
        turnRight(30, pose->heading - targetAngle);
    }
    else
    {
        turnLeft(30, targetAngle - pose->heading);
    }
}
