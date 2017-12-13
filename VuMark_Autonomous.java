package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation;
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

@Autonomous(name="VuMark_Autonomous", group ="Visual_Processing")
//@Disabled
public class VuMark_Autonomous extends LinearOpMode {
    //initiate camera
    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;

    @Override
    public void runOpMode() {

        //setup camera
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

        //prompts
        telemetry.addData(">", "Press Play to start");
        telemetry.update();

        //start run
        waitForStart();

        //Activate Relic Tracking
        relicTrackables.activate();

        //loop
        while (opModeIsActive()) {

            //get vuMark on every loop
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            //if there is anything that is recognizable
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                //put it on dashboard
                telemetry.addData("VuMark", "%s visible", vuMark);

                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
                telemetry.addData("Pose", format(pose));

                if (pose != null /*&& vuMark.equals(RelicRecoveryVuMark.LEFT)*/) {
                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                    //X, Y, and Z components of the offset of the target relative to the robot
                    double tX = trans.get(0);
                    double tY = trans.get(1);
                    double tZ = trans.get(2);
                    //Rotational components of the target relative to the robot
                    double rX = rot.firstAngle;
                    double rY = rot.secondAngle;
                    double rZ = rot.thirdAngle;
                }

            }
            else {
                telemetry.addData("VuMark", "not visible");
            }

            telemetry.update();
        }
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}
