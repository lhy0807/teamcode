package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
@Autonomous(name="AutonomousMode_Blue_Right", group="Linear Opmode")
public class AutonomousMode_Blue_Right extends LinearOpMode {

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

    private ColorSensor colorSensor;

    public int Chain_exp = 0;

    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().
                getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "Aa0j2YP/////AAAAGXZzc6vdBEenpsPVBhCR0pSDbh2nSbP0woFsOeeEcaSHmhsulEXzgAGFlGQWX/qWCqHMkq7YMGtFasFlq2RnXiFc0uUQ4XLQElRYxlSsb/Prtgt/dmrtE1ENUZBdqMq3kyE4766IAvtxTVf73erfyf0hv2IDlM/i785yySkOWUol40yPHB/x7r//Gn/OGNI6Sgf6RjaAdk702dHpE2qiE/JLRIj0XiTnZoFLgvuNcWcbJW89G6tzrYBuW+2ExZ2qW8yhB/QY8ZKl0UFi7dSPa09Zud+os8h9O+oEj+fi1S6sVK18BK7nXJQgOTpV/0UO5FPkIi1hsmZD6dlSnPbvhpQfYEJbVMc1829WOxkopAg7";
        //setup back camera
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        //import asset "Relic Vumarks"
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

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

        //init color sensor
        colorSensor = hardwareMap.get(ColorSensor.class,"color");

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
        rightfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightrear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftrear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Config mode on start
        leftChain.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightChain.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightrear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftrear.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        arm_1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        f_clawServo.scaleRange(0.3,0.9);
        b_clawServo.scaleRange(0.4,0.9);
        telemetry.addData("F_claw POS",f_clawServo.getPosition());
        telemetry.addData("B_claw POS",b_clawServo.getPosition());

        right_wheel.setPower(0);
        left_wheel.setPower(0);

        side_servo.setPosition(1);

        //Print
        telemetry.addData("Status", "init() Done");
        telemetry.update();

        relicTrackables.activate();
        //Detect Pattern
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

        waitForStart();

        while (opModeIsActive()) {

            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                //put it on dashboard
                telemetry.addData("VuMark", "%s visible", vuMark);
            }

            side_servo.setPosition(0.45);
            sleep(2000);
            //Detect Color
            int iColor = 0;
            double dRed = colorSensor.red();
            double dBlue = colorSensor.blue();
            //=1 red //=0 blue //=-1 nothing
            if(dRed>dBlue){
                iColor = 1;
                telemetry.addData("Color is Red",iColor);
            }else if(dRed<dBlue){
                iColor = -1;
                telemetry.addData("Color is Blue",iColor);
            }else{
                iColor = 0;
                telemetry.addData("No Color",iColor);
            }

            //Move and Place

            ball(iColor);

            block();


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + timer.toString());
            telemetry.addData("Motors", "leftfront (%d), leftrear (%d), rightfront (%d), rightrear (%d)", leftfront.getCurrentPosition(), leftrear.getCurrentPosition(), rightfront.getCurrentPosition(), rightrear.getCurrentPosition());
            telemetry.update();

            break;
        }
    }


    void ball(int color) {

        rightfront.setPower(0.2);
        rightrear.setPower(0.2);
        leftfront.setPower(0.2);
        leftrear.setPower(0.2);

        if (color == 1) {
            telemetry.addData("Color is Red",color);
            telemetry.update();
            rightrear.setTargetPosition(200);
            leftrear.setTargetPosition(-200);
            rightfront.setTargetPosition(200);
            leftfront.setTargetPosition(-200);
            sleep(1000);
            rightrear.setTargetPosition(0);
            leftrear.setTargetPosition(0);
            rightfront.setTargetPosition(0);
            leftfront.setTargetPosition(0);
            sleep(1000);
            side_servo.setPosition(1);
            telemetry.addData("BACK",color);
            telemetry.update();
        }
        else if (color == -1) {
            telemetry.addData("Color is Blue",color);
            telemetry.update();
            rightrear.setTargetPosition(-200);
            leftrear.setTargetPosition(200);
            rightfront.setTargetPosition(-200);
            leftfront.setTargetPosition(200);
            sleep(1000);
            rightrear.setTargetPosition(0);
            leftrear.setTargetPosition(0);
            rightfront.setTargetPosition(0);
            leftfront.setTargetPosition(0);
            sleep(1000);
            side_servo.setPosition(1);
            telemetry.addData("BACK",color);
            telemetry.update();
        }
        else {
            telemetry.addData("No Color",color);
            telemetry.update();
            rightrear.setTargetPosition(0);
            leftrear.setTargetPosition(0);
            rightfront.setTargetPosition(0);
            leftfront.setTargetPosition(0);
            sleep(1000);
            side_servo.setPosition(1);
            telemetry.addData("BACK",color);
            telemetry.update();
        }

    }

    void block() {
        rightfront.setPower(0.2);
        rightrear.setPower(0.2);
        leftfront.setPower(0.2);
        leftrear.setPower(0.2);

        rightrear.setTargetPosition(-10000);
        leftrear.setTargetPosition(-10000);
        rightfront.setTargetPosition(-10000);
        leftfront.setTargetPosition(-10000);
    }


}
