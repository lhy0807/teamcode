package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TeleOpMode_StringCar", group="Iterative Opmode")

public class TeleOpMode_StringCar extends OpMode
{
    
    //Main Timer
    private ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    //DcMotors
    private DcMotor leftfront;
    private DcMotor rightfront;
    private DcMotor leftrear;
    private DcMotor rightrear;
    private DcMotor leftMotor;
    private DcMotor rightMotor;

    //Servos
    private Servo leftServo;
    private Servo rightServo;
    private Servo frontServo;

    boolean []gamepad1_stat = new boolean[14];

    @Override
    public void init() {
        //Print
        telemetry.addData("Status", "init() Running");
        telemetry.update();
        //initiating motors (Normal)
        leftfront  = hardwareMap.get(DcMotor.class, "motor2");
        rightfront = hardwareMap.get(DcMotor.class, "motor1");
        leftrear = hardwareMap.get(DcMotor.class, "motor3");
        rightrear = hardwareMap.get(DcMotor.class,"motor4");
        leftMotor = hardwareMap.get(DcMotor.class, "leftmotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightmotor");
        leftServo = hardwareMap.get(Servo.class, "leftservo");
        rightServo = hardwareMap.get(Servo.class, "rightservo");
        frontServo = hardwareMap.get(Servo.class, "frontservo");

        //set motor direction
        leftfront.setDirection(DcMotor.Direction.REVERSE);
        leftrear.setDirection(DcMotor.Direction.REVERSE);
        rightfront.setDirection(DcMotor.Direction.FORWARD);
        rightrear.setDirection(DcMotor.Direction.FORWARD);
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        leftServo.setDirection(Servo.Direction.REVERSE);
        rightServo.setDirection(Servo.Direction.FORWARD);

        leftfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftrear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightrear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        for(int i=0;i<gamepad1_stat.length;i++){
            gamepad1_stat[i] = false;
        }

       //Print
        telemetry.addData("Status", "init() Done");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        timer.reset();
    }

    @Override
    public void start() {
        timer.reset();
        telemetry.addData("Timer", "%.3f",timer.time());
        telemetry.update();
    }

    @Override
    public void loop()   {
        /***Chasis***/
        //motor power settings
        double power_1;
        double power_2;
        double power_3;
        double power_4;
        double trim_max;
        //raw stick input (Reverse both Y axis)
        double gamepad1_X = gamepad1.left_stick_x; //leftX
        double gamepad1_Y = -gamepad1.left_stick_y; //leftY
        double gamepad1_Z = gamepad1.right_stick_x; //RightX
        double gamepad1_W = -gamepad1.right_stick_y; //RightY

        //power raw
        power_1 = Functions.MecDrive_RightFront(
                Functions.stickMod(gamepad1_X,RoboMap.bDeadzone,RoboMap.bNonLinearInput),
                Functions.stickMod(gamepad1_Y,RoboMap.bDeadzone,RoboMap.bNonLinearInput),
                Functions.stickMod(gamepad1_Z,RoboMap.bDeadzone,RoboMap.bNonLinearInput),
                Functions.stickMod(gamepad1_W,RoboMap.bDeadzone,RoboMap.bNonLinearInput)
                );
        power_2 = Functions.MecDrive_LeftFront(
                Functions.stickMod(gamepad1_X,RoboMap.bDeadzone,RoboMap.bNonLinearInput),
                Functions.stickMod(gamepad1_Y,RoboMap.bDeadzone,RoboMap.bNonLinearInput),
                Functions.stickMod(gamepad1_Z,RoboMap.bDeadzone,RoboMap.bNonLinearInput),
                Functions.stickMod(gamepad1_W,RoboMap.bDeadzone,RoboMap.bNonLinearInput)
        );
        power_3 = Functions.MecDrive_LeftRear(
                Functions.stickMod(gamepad1_X,RoboMap.bDeadzone,RoboMap.bNonLinearInput),
                Functions.stickMod(gamepad1_Y,RoboMap.bDeadzone,RoboMap.bNonLinearInput),
                Functions.stickMod(gamepad1_Z,RoboMap.bDeadzone,RoboMap.bNonLinearInput),
                Functions.stickMod(gamepad1_W,RoboMap.bDeadzone,RoboMap.bNonLinearInput)
        );
        power_4 = Functions.MecDrive_RightRear(
                Functions.stickMod(gamepad1_X,RoboMap.bDeadzone,RoboMap.bNonLinearInput),
                Functions.stickMod(gamepad1_Y,RoboMap.bDeadzone,RoboMap.bNonLinearInput),
                Functions.stickMod(gamepad1_Z,RoboMap.bDeadzone,RoboMap.bNonLinearInput),
                Functions.stickMod(gamepad1_W,RoboMap.bDeadzone,RoboMap.bNonLinearInput)
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

        /***Lifters***/

        /*//gamepad1.a press and repress
        if(gamepad1.a&&!gamepad1_stat[0]){
            gamepad1_stat[0] = true;
            if(leftMotor.getPower()!=0){
                leftMotor.setPower(0);
                rightMotor.setPower(0);
            }else{
                leftMotor.setPower(0.3);
                rightMotor.setPower(0.3);
            }
        }else if(!gamepad1.a){
            gamepad1_stat[0] = false;
        }else{}

        //gamepad1.b press and repress
        if(gamepad1.b&&!gamepad1_stat[0]){
            gamepad1_stat[1] = true;
            if(leftMotor.getPower()!=0){
                leftMotor.setPower(0);
                rightMotor.setPower(0);
            }else{
                leftMotor.setPower(-0.3);
                rightMotor.setPower(-0.3);
            }
        }else if(!gamepad1.b){
            gamepad1_stat[1] = false;
        }else{}*/
        if(gamepad1.a){
            leftMotor.setPower(-1);
            rightMotor.setPower(-1);
        }else if(gamepad1.b){
            leftMotor.setPower(1);
            rightMotor.setPower(1);
        }else{
            leftMotor.setPower(0);
            rightMotor.setPower(0);
        }

        //experimental
        if(gamepad1.x&&!gamepad1_stat[0]){
            gamepad1_stat[0] = true;
            if(leftServo.getPosition()!=0){
                leftServo.setPosition(0);
                rightServo.setPosition(0);
            }else{
                leftServo.setPosition(0.4);
                rightServo.setPosition(0.4);
            }
        }else if(!gamepad1.x){
            gamepad1_stat[0] = false;
        }else{}


        frontServo.setPosition(1);





        //put data into dashboard
        telemetry.addData("Config_Main_Timer", "Run Time: " + timer.toString());
        telemetry.addData("Config_trim",trim_max);
         //telemetry.addData("gamepad1.atRest()",gamepad1.atRest());
        telemetry.addData("Config_gamepad1.Input","LX (%.2f), LY (%.2f), RX (%.2f), RY (%.2f)",
                gamepad1_X,gamepad1_Y,gamepad1_Z,gamepad1_W);
        telemetry.addData("Config_MotorPower","1 (%.2f), 2 (%.2f), 3 (%.2f),4 (%.2f)",
                power_1,power_2,power_3,power_4);

    }

    @Override
    public void stop() {
        leftfront.setPower(0);
        leftrear.setPower(0);
        rightfront.setPower(0);
        rightrear.setPower(0);
        //leftMotor.setPower(0);
        //rightMotor.setPower(0);
        telemetry.clearAll();
        telemetry.addData("Status","Stop Enforced");
        telemetry.update();
    }

}