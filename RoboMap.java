package org.firstinspires.ftc.teamcode;

public class RoboMap {
    //Robot Motor Mapping (Not used currently)
    protected static final int RightFront = 1;
    protected static final int LeftFront = 2;
    protected static final int LeftRear = 3;
    protected static final int RightRear = 4;

    //Joystick Config
    //DO NOT CHANGE!!!! DO NOT USE ANY OTHER SCALING METHOD!!!!
    protected static final boolean bDeadzone = false; //if deadzone is activated
    protected static final double dDeadzone = 0.02; //deadzone range if activated (MUST be POSITIVE)
    protected static final boolean bNonLinearInput = false; //non-linear input method

    //Mecanum Driving Params - SEE teamcode.Functions
    protected static final double pZ = 1; //right X rotated
    protected static final double pY = 1; //left Y drive
    protected static final double pX = 1; //left X shift
    protected static final double pW = 1; //right Y drive

    //todo AutonomousMode_PID_Test Config [IN PROGRESS]
    protected static final double Kp = 0.5;
    protected static final double Ki = 0;
    protected static final double Kd = 0;
    protected static final double Error_Acceptable = 2.5;//In degrees
}
