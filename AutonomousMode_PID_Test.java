package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="AutonomousMode", group="Linear Opmode")
@Disabled
public class AutonomousMode_PID_Test extends LinearOpMode {

    private DcMotor leftfront;
    private DcMotor rightfront;
    private DcMotor leftrear;
    private DcMotor rightrear;
    private DcMotor testMotor;
    private ModernRoboticsI2cGyro gyro;
    private double []dError = new double[4];

    private ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        //Hardware initiate
        leftfront  = hardwareMap.get(DcMotor.class, "l_f");
        rightfront = hardwareMap.get(DcMotor.class, "r_b");
        leftrear = hardwareMap.get(DcMotor.class, "l_b");
        rightrear = hardwareMap.get(DcMotor.class,"r_b");
        gyro = hardwareMap.get(ModernRoboticsI2cGyro.class,"gyro");

        //set motors direction
        leftfront.setDirection(DcMotor.Direction.FORWARD);
        leftrear.setDirection(DcMotor.Direction.FORWARD);
        rightfront.setDirection(DcMotor.Direction.REVERSE);
        rightrear.setDirection(DcMotor.Direction.REVERSE);

        //reset motor mode
        leftfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftrear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightrear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //calibrate gyro
        telemetry.addData("Gyro_Stat","Caliberating...");
        telemetry.update();
        gyro.calibrate();
        while(gyro.isCalibrating()&&!isStopRequested()){
            sleep(50);
            idle();
        }
        telemetry.addData("Gyro_Stat","^_^");
        telemetry.update();

        //wait for start
        waitForStart();

        //reset after start
        timer.reset();
        gyro.resetZAxisIntegrator();

        sleep(3000);
        autoTurning(-90,30);
        autoTurning(90,30);


        while (opModeIsActive()) {
            idle();
        }

    }

    /**
     * Runnable Turning
     * */
    private void autoTurning(double targetAngle, double timeOut){
        telemetry.addData("autoTurning","isMoving");
        ElapsedTime turnTimeoutTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        while(opModeIsActive()&&(turnTimeoutTimer.time()<=timeOut&&!isLocked(targetAngle))) {
            telemetry.update();
        }
        telemetry.addData("autoTurning","Completed");
    }
    private void autoTurning(double targetAngle){
        telemetry.addData("autoTurning","isMoving");
        ElapsedTime turnTimeoutTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        while(opModeIsActive()&&!isLocked(targetAngle)) {
            telemetry.update();
        }
        telemetry.addData("autoTurning","Completed");
    }

    /**
     * This is a Processable method!!! The return is only for Reading stat purpose
     * Should set MotorMode before using this method
     * @param targetAngle targeted Error to Mecanum Turn (sim-stick)
     * @return if the heading is on targetAngle
     */
    private boolean isLocked(double targetAngle){
        boolean onLock = false;
        double curError = getAngluarError(targetAngle);
        double []power = new double[4];

        //whether in range or not
        if(Math.abs(curError)<=RoboMap.Error_Acceptable){
            onLock = true;
            for(int i=0; i<power.length; i++){
                power[i]=0;
            }
        }else{
            double dtemp = p_TurnInput(curError,RoboMap.Kp);
            power[0] = Functions.MecDrive_RightFront(
                    Functions.stickMod(0,false,false),
                    Functions.stickMod(0,false,false),
                    Functions.stickMod(dtemp,false,false),0
            );
            power[1] = Functions.MecDrive_LeftFront(
                    Functions.stickMod(0,false,false),
                    Functions.stickMod(0,false,false),
                    Functions.stickMod(dtemp,false,false),0
            );
            power[2] = Functions.MecDrive_LeftRear(
                    Functions.stickMod(0,false,false),
                    Functions.stickMod(0,false,false),
                    Functions.stickMod(dtemp,false,false),0
            );
            power[3] = Functions.MecDrive_RightRear(
                    Functions.stickMod(0,false,false),
                    Functions.stickMod(0,false,false),
                    Functions.stickMod(dtemp,false,false),0
            );
        }

        //set power
        leftfront.setPower(power[1]);
        leftrear.setPower(power[2]);
        rightfront.setPower(power[0]);
        rightrear.setPower(power[3]);

        //todo telemetry.addData();
        return onLock;
    }

    /**
     * get angular error (not reset so universal)
     * */
    private double getAngluarError(double targetAngle){
        double dtemp = targetAngle - gyro.getIntegratedZValue();
        if(dtemp>=-180&&dtemp<=180){
            return dtemp;
        }else{
            return (dtemp>180)?(dtemp-360):(dtemp+360);
        }
    }

    /**
     * Motor input returned upon Joystick simulation
     * Using only the Kp coefficient
     */
    private double p_TurnInput(double angluarError, double Kp){
        double dtemp = angluarError*(Math.PI/180)*Kp;
        if(dtemp>=-1&&dtemp<=1){
            return dtemp;
        }else{
            return (dtemp>1)?(1):(-1);
        }
    }

    //todo PID full method
}
