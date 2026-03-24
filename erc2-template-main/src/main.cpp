#include <FEH.h>
#include <Arduino.h>
// githubtest

// Motor and encoder declarations
FEHMotor leftMotor(FEHMotor::Motor2, 6.0);
FEHMotor rightMotor(FEHMotor::Motor0, 6.0);
DigitalEncoder leftEncoder(FEHIO::Pin10);
DigitalEncoder rightEncoder(FEHIO::Pin8);
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
const float LEFT_OPTOSENSOR_THRESHOLD = 4;   // threshold value for left optosensor on the line (black line will have a value above this threshold, white background will have a value below this threshold)
const float MIDDLE_OPTOSENSOR_THRESHOLD = 4; // threshold value for middle optosensor on the line (black line will have a value above this threshold, white background will have a value below this threshold)
const float RIGHT_OPTOSENSOR_THRESHOLD = 4;  // threshold value for right optosensor on the line (black line will have a value above this threshold, white background will have a value below this threshold)
const float CDS_CELL_RED_THRESHOLD = 0.48;   // threshold value for cds cell to determine if the humidifier light is red or blue (red light will have a value below this threshold, blue light will have a value above this threshold)

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
void goForwardPID(float inch_per_sec, float distance)
{
    const float P_CONSTANT = 0.75; // P term, the higher the value the more aggressively the controller will correct for current error
    const float I_CONSTANT = 0.05; // I term, the higher the value the more it will correct for accumulated past error (helps with systematic bias like weight imbalance)
    const float D_CONSTANT = 0.25; // D term, the higher the value the more it will correct for rate of change of error (helps with preventing overshoot)
    const float SLEEP_TIME = 0.1;  // Sleep time between iterations, 0.1 to 0.2 seconds is generally safe

    // 2. 计算目标总编码器计数
    int targetCounts = distance / INCHES_PER_COUNT;

    // 3. 初始化 PID 所需的变量
    float left_power = inch_per_sec; // 初始给一个基础功率猜测值
    float right_power = inch_per_sec;

    float left_error = 0.0, right_error = 0.0;
    float left_error_sum = 0.0, right_error_sum = 0.0;
    float left_last_error = 0.0, right_last_error = 0.0;

    int left_last_counts = 0, right_last_counts = 0;
    float last_time = TimeNow();

    // 4. 重置编码器
    leftEncoder.ResetCounts();
    rightEncoder.ResetCounts();

    // 为了避免第一次循环时时间差 (Delta Time) 为 0 导致除以 0 的错误，先短暂等待 [cite: 111, 120]
    Sleep(SLEEP_TIME);

    // 5. 主控制循环：当左右编码器的平均值还没有达到目标距离时，持续调节
    while ((abs(leftEncoder.Counts()) + abs(rightEncoder.Counts())) / 2 < targetCounts)
    {
        float current_time = TimeNow();
        float delta_time = current_time - last_time;

        // 安全保护：防止极小概率下的除零崩溃
        if (delta_time <= 0.0)
            delta_time = 0.001;

        int current_left_counts = abs(leftEncoder.Counts());
        int current_right_counts = abs(rightEncoder.Counts());

        int delta_left_counts = current_left_counts - left_last_counts;
        int delta_right_counts = current_right_counts - right_last_counts;

        // 步骤 A：计算当前的实际速度 (Actual Velocity) [cite: 41, 125]
        float left_actual_speed = INCHES_PER_COUNT * ((float)delta_left_counts / delta_time);
        float right_actual_speed = INCHES_PER_COUNT * ((float)delta_right_counts / delta_time);

        // 步骤 B：计算误差 (Error) = 期望速度 - 实际速度 [cite: 49, 126]
        left_error = inch_per_sec - left_actual_speed;
        right_error = inch_per_sec - right_actual_speed;

        // 步骤 C：累加误差 (Error Sum) 供积分项使用 [cite: 63, 127]
        left_error_sum += left_error;
        right_error_sum += right_error;

        // 步骤 D：分别计算 P、I、D 三个补偿项 [cite: 128, 129, 130]
        float left_P = left_error * P_CONSTANT;
        float left_I = left_error_sum * I_CONSTANT;
        float left_D = (left_error - left_last_error) * D_CONSTANT;

        float right_P = right_error * P_CONSTANT;
        float right_I = right_error_sum * I_CONSTANT;
        float right_D = (right_error - right_last_error) * D_CONSTANT;

        // 步骤 E：计算新的电机输出功率 = 旧功率 + P + I + D [cite: 54, 134]
        left_power = left_power + left_P + left_I + left_D;
        right_power = right_power + right_P + right_I + right_D;

        // 步骤 F：限制功率边界，防止功率超出 0-100% 的合法区间
        if (left_power > 100.0)
            left_power = 100.0;
        if (left_power < 5.0)
            left_power = 5.0; // 留一点底线防止卡死
        if (right_power > 100.0)
            right_power = 100.0;
        if (right_power < 5.0)
            right_power = 5.0;

        // 步骤 G：赋值给电机。根据你的硬件结构，左轮设为负值以向前行驶 [cite: 148]
        leftMotor.SetPercent(-left_power);
        rightMotor.SetPercent(right_power);

        // 步骤 H：保存当前状态，供下一次循环的微积分计算使用 [cite: 131]
        left_last_error = left_error;
        right_last_error = right_error;
        left_last_counts = current_left_counts;
        right_last_counts = current_right_counts;
        last_time = current_time;

        // 等待一段时间再进行下一次采样和计算 [cite: 139]
        Sleep(SLEEP_TIME);
    }

    // 循环结束，到达目标距离，停止电机 [cite: 112]
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
}

int readLineState()
{
    float leftValue = optosensorLeft.Value();
    float middleValue = optosensorMiddle.Value();
    float rightValue = optosensorRight.Value();

    return ((leftValue > leftLineThreshold ? 1 : 0) << 2) | ((middleValue > middleLineThreshold ? 1 : 0) << 1) | ((rightValue > rightLineThreshold ? 1 : 0) << 0);
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
void followLine(int percent) // currently working, need to add PID
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
        LCD.Clear();
        LCD.WriteLine(optosensorLeft.Value());
        LCD.WriteLine(optosensorMiddle.Value());
        LCD.WriteLine(optosensorRight.Value());
        LCD.WriteLine(readLineState());

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

void followLinePID(float base_percent)
{
    const float LOST_LINE_TIMEOUT = 0.9;
    const float INTERSECTION_HOLD_TIME = 0.12;

    float lostLineTimer = 0.0;
    float allOnTimer = 0.0;
    bool isLost = false;
    bool allOn = false;

    //PID constants
    const float Kp = 13; // Kp, base speed variance for every error, change this first if the robot is not responsive enough or too responsive
    const float Ki = 0.02; // Ki, integral term: corrects long-term deviation (e.g., motor imbalance)
    const float Kd = 0.7; // Kd, derivative term: predicts trend, resists inertia during turns, reduces left-right "drawing"

    float error = 0.0;
    float last_error = 0.0;
    float integral = 0.0;
    float derivative = 0.0;

    const float MAX_SPEED = base_percent; //base speed for straight line
    const float MID_SPEED = 35; // base speed for inner wheels when turning
    const float MIN_SPEED = 25; //base speed for inner wheels when sharp turning

    while (1)
    {
        //LCD.Clear();
        LCD.WriteLine(error);

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
        if(error==0){
            current_base_speed = MAX_SPEED;
        }else if(abs(error)==1){
            current_base_speed = MID_SPEED;
        }else{
            current_base_speed = MIN_SPEED;
        }

        //PID calculations
        if (error == 0)
        {
            integral = 0;
        }
        else
        {
            integral = integral + error;
        }

        derivative = error - last_error;

        //overall adjustments based on PID terms
        float adjustment = (Kp * error) + (Ki * integral) + (Kd * derivative);

        // making adjustment to motor power
        // if error is positive, means we need to turn left, so left motor should be slower than right motor
        float left_power = current_base_speed - adjustment;
        float right_power = current_base_speed + adjustment;

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
    // calibrateLineThresholds(0.12);
    followLinePID(50);
    // goForwardPID(10,30);
}
// min at full speed 884
// max at full speed 1876
// stop point 1450
// clkwise width: 1450-1380=70
// counterclkwise width: 1526-1450=76