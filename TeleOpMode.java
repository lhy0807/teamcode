package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TeleOpMode", group="Iterative Opmode")

public class TeleOpMode extends OpMode
{
    
    //Main Timer
    private ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    //DcMotors
    private DcMotor leftfront;
    private DcMotor rightfront;
    private DcMotor leftrear;
    private DcMotor rightrear;
    private DcMotor leftChain;
    private DcMotor rightChain;
    private DcMotor arm_1;
    private DcMotor arm_2;

    //Servos
    private Servo f_clawServo;
    private Servo b_clawServo;
    private Servo arm_servo_1;
    private Servo arm_servo_2;
    private Servo side_servo;

    //wheel_servos
    private CRServo right_wheel;
    private CRServo left_wheel;

    public int Chain_exp = 0;

    @Override
    public void init() {
        //Print
        telemetry.addData("Status", "init() Running");
        telemetry.update();
        //initiating motors (Normal)
        leftfront  = hardwareMap.get(DcMotor.class, "l_f");
        rightfront = hardwareMap.get(DcMotor.class, "r_f");
        leftrear = hardwareMap.get(DcMotor.class, "l_b");
        rightrear = hardwareMap.get(DcMotor.class,"r_b");
        leftChain = hardwareMap.get(DcMotor.class,"leftchain");
        rightChain = hardwareMap.get(DcMotor.class,"rightchain");
        arm_1 = hardwareMap.get(DcMotor.class,"arm_1");
        arm_2 = hardwareMap.get(DcMotor.class,"arm_2");

        //initiating Servos (Normal)
        f_clawServo = hardwareMap.get(Servo.class,"fclaw");
        b_clawServo = hardwareMap.get(Servo.class,"bclaw");
        arm_servo_1 = hardwareMap.get(Servo.class,"armservo1");
        arm_servo_2 = hardwareMap.get(Servo.class,"armservo2");
        side_servo = hardwareMap.get(Servo.class,"side");


        //init CRServos
        right_wheel = hardwareMap.get(CRServo.class,"rwheel");
        left_wheel = hardwareMap.get(CRServo.class,"lwheel");

        //set motor direction
        leftfront.setDirection(DcMotor.Direction.FORWARD);
        leftrear.setDirection(DcMotor.Direction.FORWARD);
        rightfront.setDirection(DcMotor.Direction.REVERSE);
        rightrear.setDirection(DcMotor.Direction.REVERSE);
        leftChain.setDirection(DcMotor.Direction.FORWARD);
        rightChain.setDirection(DcMotor.Direction.REVERSE);
        right_wheel.setDirection(CRServo.Direction.FORWARD);
        left_wheel.setDirection(CRServo.Direction.REVERSE);
        arm_1.setDirection(CRServo.Direction.FORWARD);
        arm_2.setDirection(CRServo.Direction.REVERSE);


        //config encoder run modes
        leftChain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightChain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Config mode on start
        leftChain.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightChain.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        arm_1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightrear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftrear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        f_clawServo.scaleRange(0.4,0.9);
        b_clawServo.scaleRange(0.4,0.9);
        telemetry.addData("F_claw POS",f_clawServo.getPosition());
        telemetry.addData("B_claw POS",b_clawServo.getPosition());

        right_wheel.setPower(0);
        left_wheel.setPower(0);

        side_servo.setPosition(1);
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

        leftChain.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightChain.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Encoder position
        int leftChain_Pos = leftChain.getCurrentPosition();
        int rightChain_Pos = rightChain.getCurrentPosition();
        telemetry.addData("Left_Pos:",leftChain_Pos);
        telemetry.addData("Right_Pos:",rightChain_Pos);
        telemetry.addData("Exp_Pos:",Chain_exp);

        //motor power settings
        double power_1;
        double power_2;
        double power_3;
        double power_4;
        //raw stick input (Reverse both Y axis)
        boolean gamepad2_a = gamepad2.a;
        boolean gamepad2_b = gamepad2.b;
        boolean gamepad1_x = gamepad1.x;
        double f_gamepad2_servo = 1-gamepad2.right_trigger;
        double b_gamepad2_servo = gamepad2.left_trigger;
        boolean gamepad1_arm_l = gamepad1.left_bumper;
        boolean gamepad1_arm_r = gamepad1.right_bumper;
        boolean gamepad1_arm_servo1_u = gamepad1.dpad_up;
        boolean gamepad1_arm_servo1_d = gamepad1.dpad_down;
        boolean gamepad1_arm_servo2_l = gamepad1.dpad_left;
        boolean gamepad1_arm_servo2_r = gamepad1.dpad_right;


        double r = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        leftfront.setPower(v1);
        rightfront.setPower(v2);
        leftrear.setPower(v3);
        rightrear.setPower(v4);

        telemetry.addData("leftfront:",v1);
        telemetry.addData("rightfront:",v2);
        telemetry.addData("leftrear:",v3);
        telemetry.addData("rightrear:",v4);

        //chain mapped on button A&B on gamepad1
        double dChainSpeed = 0.3;

        if (Math.abs(leftChain_Pos-rightChain_Pos)>40) {
            leftChain.setTargetPosition((leftChain_Pos+rightChain_Pos)/2);
            leftChain.setPower(dChainSpeed);
            rightChain.setTargetPosition((leftChain_Pos+rightChain_Pos)/2);
            rightChain.setPower(dChainSpeed);
        }
        else if (Math.abs(leftChain_Pos-rightChain_Pos)<10 && !gamepad2_a && !gamepad2_b){
            leftChain.setPower(0);
            rightChain.setPower(0);
        }
        else {
            if (gamepad2_a) {
                Chain_exp = leftChain_Pos - 200;
                leftChain.setTargetPosition(Chain_exp);
                leftChain.setPower(dChainSpeed);
                rightChain.setTargetPosition(Chain_exp);
                rightChain.setPower(dChainSpeed);
            }
            else if (gamepad2_b) {
                Chain_exp = leftChain_Pos + 200;
                leftChain.setTargetPosition(Chain_exp);
                leftChain.setPower(dChainSpeed);
                rightChain.setTargetPosition(Chain_exp);
                rightChain.setPower(dChainSpeed);
            }
            else {
                leftChain.setTargetPosition(Chain_exp);
                leftChain.setPower(dChainSpeed);
                rightChain.setTargetPosition(Chain_exp);
                rightChain.setPower(dChainSpeed);
            }
        }

        //CRServos
        if (gamepad1_x) {
            left_wheel.setPower(-1);
            right_wheel.setPower(-1);
        }
        else {
            left_wheel.setPower(1);
            right_wheel.setPower(1);
        }

        //Claw Servos

        f_clawServo.setPosition(f_gamepad2_servo);
        b_clawServo.setPosition(b_gamepad2_servo);

        telemetry.addData("f_claw_pos:",f_clawServo.getPosition());
        telemetry.addData("b_claw_pos:",b_clawServo.getPosition());

        //Long Arm
        if (gamepad1_arm_l) {
            arm_1.setPower(0.8);
            arm_2.setPower(0.8);
        }
        else if (gamepad1_arm_r) {
            arm_1.setPower(-0.8);
            arm_2.setPower(-0.8);
        }
        else {
            arm_1.setPower(0);
            arm_2.setPower(0);
        }


        //put data into dashboard
        telemetry.addData("Config_Main_Timer", "Run Time: " + timer.toString());
        telemetry.addData("Config_MotorPower","1 (%.2f), 2 (%.2f), 3 (%.2f),4 (%.2f)",
                v1,v2,v3,v4);

    }

    @Override
    public void stop() {
        leftfront.setPower(0);
        leftrear.setPower(0);
        rightfront.setPower(0);
        rightrear.setPower(0);
        leftChain.setPower(0);
        rightChain.setPower(0);
        left_wheel.setPower(0);
        right_wheel.setPower(0);
        telemetry.clearAll();
        telemetry.addData("Status","Stop Enforced");
        telemetry.update();
    }

}