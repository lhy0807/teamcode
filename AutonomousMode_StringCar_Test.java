package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.VuMarkTemplate;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.concurrent.TimeUnit;

@Autonomous(name="AutonomousMode_StringCar_Test", group="Linear Opmode")

public class AutonomousMode_StringCar_Test extends LinearOpMode {

    private DcMotor leftfront;
    private DcMotor rightfront;
    private DcMotor leftrear;
    private DcMotor rightrear;
    //private DcMotor leftMotor;
    //private DcMotor rightMotor;

    //initiate sensory input method
    private ModernRoboticsI2cGyro gyro;
    private ColorSensor colorSensor;
    //initiate camera
    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;
    //open timer
    private ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    //def config booleans
    private final boolean isRedAlliance = true;
    private final boolean useGyro = false;
    private final boolean useColor = false;
    private final boolean useEncoder = true;

    @Override
    public void runOpMode() {
        /***Hardware initiate***/
        //initiating motors (Normal)
        leftfront  = hardwareMap.get(DcMotor.class, "motor2");
        rightfront = hardwareMap.get(DcMotor.class, "motor1");
        leftrear = hardwareMap.get(DcMotor.class, "motor3");
        rightrear = hardwareMap.get(DcMotor.class,"motor4");
        //leftMotor = hardwareMap.get(DcMotor.class, "leftmotor");
        //rightMotor = hardwareMap.get(DcMotor.class, "rightmotor");

        //set motor direction
        leftfront.setDirection(DcMotor.Direction.REVERSE);
        leftrear.setDirection(DcMotor.Direction.REVERSE);
        rightfront.setDirection(DcMotor.Direction.FORWARD);
        rightrear.setDirection(DcMotor.Direction.FORWARD);
        //leftMotor.setDirection(DcMotor.Direction.FORWARD);
        //rightMotor.setDirection(DcMotor.Direction.REVERSE);

        //setup camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().
                getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey =
                "Aa0j2YP/////AAAAGXZzc6vdBEenpsPVBhCR0pSDbh2nSbP0woFsOeeEcaSHmhsulEXzgAGFlGQWX/qWCqHMkq7YMGtFasFlq2RnXiFc0uUQ4XLQElRYxlSsb/Prtgt/dmrtE1ENUZBdqMq3kyE4766IAvtxTVf73erfyf0hv2IDlM/i785yySkOWUol40yPHB/x7r//Gn/OGNI6Sgf6RjaAdk702dHpE2qiE/JLRIj0XiTnZoFLgvuNcWcbJW89G6tzrYBuW+2ExZ2qW8yhB/QY8ZKl0UFi7dSPa09Zud+os8h9O+oEj+fi1S6sVK18BK7nXJQgOTpV/0UO5FPkIi1hsmZD6dlSnPbvhpQfYEJbVMc1829WOxkopAg7";
        //using back camera on phonr
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        //import asset "Relic Vumarks"
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");
        telemetry.addData("Camera_Stat", "^_^");
        telemetry.update();

        //calibrate gyro
        /*if(useGyro){
            gyro = hardwareMap.get(ModernRoboticsI2cGyro.class,"gyro");
            telemetry.addData("Gyro_Stat","Caliberating...");
            telemetry.update();
            gyro.calibrate();
            while(gyro.isCalibrating()&&!isStopRequested()){
                sleep(50);
                idle();
            }
            telemetry.addData("Gyro_Stat","^_^");
            telemetry.update();
        }*/

        if(useColor){
            colorSensor = hardwareMap.get(ColorSensor.class,"color");
            telemetry.addData("ColorSensor", "^_^");
            telemetry.update();
        }

        if(useEncoder){
            leftfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftrear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightrear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            telemetry.addData("Encoder_Stat", "^_^");
            telemetry.update();
        }

        //todo wait for start marker
        waitForStart();
        timer.reset();
        leftfront.setPower(0);
        leftrear.setPower(0);
        rightfront.setPower(0);
        rightrear.setPower(0);
        //gyro.resetZAxisIntegrator();
        relicTrackables.activate();

        //todo Marker config Vumark
        int vuMark_Marker = vumarkRecog(relicTemplate,2);
        telemetry.addData("vuMark_Marker",vuMark_Marker);
        telemetry.update();

        encoderRun(1120,1120,1120,1120,0.8,10,0.25);
        encoderRun(-1120,1120,-1120,1120,0.8,10,0.25);
        encoderRun(-1120,1120,1120,-1120,0.8,10,0.25);

        analogRun(0,-0.5,0,0.5);

        /*
        sleep(1000);
        int iColor=0;

        while(opModeIsActive()){
            double dRed = colorSensor.red();
            double dBlue = colorSensor.blue();
            //=1 red //=0 blue //=-1 nothing
            if(dRed>dBlue){
                iColor = 1;
            }else if(dRed<dBlue){
                iColor = -1;
            }else{
                iColor = 0;
            }

            telemetry.addData("R", colorSensor.red());
            telemetry.addData("B ", colorSensor.blue());
            telemetry.addData("1red0blue",iColor);
            telemetry.update();
            //idle();
            if(iColor!=0){
                break;
            }
        }
        if(isRedAlliance){
            if(iColor==1){
                analogRun(0,0,1,0.3);
                analogRun(0,0,-1,0.3);
            }else if(iColor==-1){
                analogRun(0,0,-1,0.3);
                analogRun(0,0,1,0.3);
            }else{}
        }else{
            if(iColor==-1){
                analogRun(0,0,1,0.3);
                analogRun(0,0,-1,0.3);
            }else if(iColor==1){
                analogRun(0,0,-1,0.3);
                analogRun(0,0,1,0.3);
            }else{}
        }

        sleep(1000);
        analogRun(0,0.5,0,1.5);
        */



    }



    //todo below is the methods that are separated from runable main

    /**
     * Vumark Scanning
     */
    private int vumarkRecog(VuforiaTrackable relicTemplate, int timeOut){
        int iTemp = 0;
        ElapsedTime vumarkTimer = new ElapsedTime();
        vumarkTimer.reset();
        while ((opModeIsActive()&&vumarkTimer.seconds()<=timeOut)&&(iTemp!=0)) {
            //get vuMark on every loop
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            //if there is anything that is recognizable
            if(vuMark.equals(RelicRecoveryVuMark.LEFT)){
                iTemp = 1;
                telemetry.addData("VuMark", "LEFT");
                telemetry.update();
            }else if (vuMark.equals(RelicRecoveryVuMark.CENTER)){
                iTemp = 2;
                telemetry.addData("VuMark", "CENTER");
                telemetry.update();
            }else if(vuMark.equals(RelicRecoveryVuMark.RIGHT)){
                iTemp = 3;
                telemetry.addData("VuMark", "RIGHT");
                telemetry.update();
            }else{
                iTemp = 0;
                telemetry.addData("VuMark", "not visible");
                telemetry.update();
            }
        }
        //return marker if expired
        return iTemp;
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

    private void encoderRun(int target1, int target2, int target3, int target4, double speed, double timeOut, double sleepTime){
        ElapsedTime encoderTimer = new ElapsedTime();

        if(opModeIsActive()){
            leftfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftrear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightrear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftfront.setTargetPosition(leftfront.getCurrentPosition()+target2);
            leftrear.setTargetPosition(leftfront.getCurrentPosition()+target3);
            rightfront.setTargetPosition(leftfront.getCurrentPosition()+target1);
            rightrear.setTargetPosition(leftfront.getCurrentPosition()+target4);

            leftfront.setPower(Math.abs(speed));
            leftrear.setPower(Math.abs(speed));
            rightfront.setPower(Math.abs(speed));
            rightrear.setPower(Math.abs(speed));

            encoderTimer.reset();
            while(opModeIsActive()&&((encoderTimer.seconds()<=timeOut)&&
                    ((leftfront.isBusy()||leftrear.isBusy())||
                    (rightrear.isBusy()||rightfront.isBusy())))){
                //nothing...just to make sure everything is finished before going on
            }

            leftfront.setPower(0);
            leftrear.setPower(0);
            rightfront.setPower(0);
            rightrear.setPower(0);
            leftfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftrear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightrear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            telemetry.addData("encoderRun","Completed");
            telemetry.update();

            sleep((long)sleepTime*1000);
        }

    }


    /**
     * Runable Speed Time analog run
     */
    private void analogRun(double analog_x, double analog_y, double analog_z, double timeOut){
        telemetry.addData("analogRun","isMoving");
        telemetry.update();
        ElapsedTime analogRunTimeoutTimer = new ElapsedTime();
        while(opModeIsActive()&&analogRunTimeoutTimer.seconds()<=timeOut) {
            double power_1;
            double power_2;
            double power_3;
            double power_4;
            double trim_max;

            power_1 = Functions.MecDrive_RightFront(
                    Functions.stickMod(analog_x,false,false),
                    Functions.stickMod(analog_y,false,false),
                    Functions.stickMod(analog_z,false,false),
                    Functions.stickMod(0,false,false)
            );
            power_2 = Functions.MecDrive_LeftFront(
                    Functions.stickMod(analog_x,false,false),
                    Functions.stickMod(analog_y,false,false),
                    Functions.stickMod(analog_z,false,false),
                    Functions.stickMod(0,false,false)
            );
            power_3 = Functions.MecDrive_LeftRear(
                    Functions.stickMod(analog_x,false,false),
                    Functions.stickMod(analog_y,false,false),
                    Functions.stickMod(analog_z,false,false),
                    Functions.stickMod(0,false,false)
            );
            power_4 = Functions.MecDrive_RightRear(
                    Functions.stickMod(analog_x,false,false),
                    Functions.stickMod(analog_y,false,false),
                    Functions.stickMod(analog_z,false,false),
                    Functions.stickMod(0,false,false)
            );

            //scale output so that power is scaled with maximum of 1
            trim_max = Math.max(Math.max(Math.max(Math.abs(power_1),Math.abs(power_2)),
                    Math.max(Math.abs(power_3),Math.abs(power_4))),1);
            power_1 /= trim_max;
            power_2 /= trim_max;
            power_3 /= trim_max;
            power_4 /= trim_max;

            //set motor power
            leftfront.setPower(power_2);
            leftrear.setPower(power_3);
            rightfront.setPower(power_1);
            rightrear.setPower(power_4);

            telemetry.update();
        }
        leftfront.setPower(0);
        leftrear.setPower(0);
        rightfront.setPower(0);
        rightrear.setPower(0);
        telemetry.addData("analogRun","Completed");
        telemetry.update();
    }

    /**
     * Runnable Turning

    private void autoTurning(double targetAngle, double timeOut){
        telemetry.addData("autoTurning","isMoving");
        telemetry.update();
        ElapsedTime turnTimeoutTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        while(opModeIsActive()&&(turnTimeoutTimer.time()<=timeOut&&!isLocked(targetAngle))) {
            telemetry.update();
        }
        telemetry.addData("autoTurning","Completed");
        telemetry.update();
    }
    private void autoTurning(double targetAngle){
        telemetry.addData("autoTurning","isMoving");
        telemetry.update();
        ElapsedTime turnTimeoutTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        while(opModeIsActive()&&!isLocked(targetAngle)) {
            telemetry.update();
        }
        telemetry.addData("autoTurning","Completed");
        telemetry.update();
    }

    *
     * This is a Processable method!!! The return is only for Reading stat purpose
     * Should set MotorMode before using this method
     * @param targetAngle targeted Error to Mecanum Turn (sim-stick)
     * @return if the heading is on targetAngle
     *
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

    //get angular error (not reset so universal)
    private double getAngluarError(double targetAngle){
        double dtemp = targetAngle - gyro.getIntegratedZValue();
        if(dtemp>=-180&&dtemp<=180){
            return dtemp;
        }else{
            return (dtemp>180)?(dtemp-360):(dtemp+360);
        }
    }

    //Motor input returned upon Joystick simulation
    //Using only the Kp coefficient
    private double p_TurnInput(double angluarError, double Kp){
        double dtemp = angluarError*(Math.PI/180)*Kp;
        if(dtemp>=-1&&dtemp<=1){
            return dtemp;
        }else{
            return (dtemp>1)?(1):(-1);
        }
    }*/

    //todo PID full method
}
