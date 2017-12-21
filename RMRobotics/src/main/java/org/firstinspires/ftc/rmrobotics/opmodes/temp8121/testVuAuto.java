package org.firstinspires.ftc.rmrobotics.opmodes.temp8121;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

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

/**
 * Created by rotom on 10/17/2017.
 */

@Autonomous(name="testVuAuto", group ="woRMholeConfig")
public class testVuAuto extends LinearOpMode{
    public static final String TAG = "Vuforia VuMark Sample";
    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;
    @Override public void runOpMode() {


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "ATsODcD/////AAAAAVw2lR...d45oGpdljdOh5LuFB9nDNfckoxb8COxKSFX";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");
//        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();
        relicTrackables.activate();

        //0,0,0 is front left bottom corner. phone is a matrix of where the phone is relative to the robot
        OpenGLMatrix phone2bot = OpenGLMatrix
                .translation(3.75f, 5.37f, 6.248f) //temporary and arbitrary values. goal is roughly closer to the front, far right side, near the bottom ish
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ,
                        AngleUnit.DEGREES, 0, 0, 0));
        //theoretically should have phone upright, screen facing out towards the right
//        RobotLog.ii(TAG, "phone=%s", format(phoneLocationOnRobot));

        //trans: bottom left of field is 0,0. red bottom picture is (0,34,0) in inches. blue bottom (144,34,0) red top (0,106,0) blue top (144,106,0)
        //rot: (0,0,90) (0,0,-90), (0,0,0), (0,0,0)
        //this program is testing red top
        OpenGLMatrix pic2field = OpenGLMatrix
                .translation(0,106,0) //temporary and arbitrary values. goal is roughly closer to the front, far right side, near the bottom ish
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ,
                        AngleUnit.DEGREES, 0, 0, 90));
        while (opModeIsActive()) {
            OpenGLMatrix bot2field = new OpenGLMatrix();
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                //telemetry.addData("VuMark", "%s visible", vuMark);
                OpenGLMatrix pic2phone = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
                //telemetry.addData("Pose", format(pic2phone));
                if (pic2phone != null) {
                    OpenGLMatrix pic2bot = pic2phone.multiplied(phone2bot);
                    OpenGLMatrix bot2pic = pic2bot.transposed();
                    bot2field = bot2pic.multiplied(pic2field);
//                    VectorF trans = pose.getTranslation();
//                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
//                    double tX = trans.get(0);
//                    double tY = trans.get(1);
//                    double tZ = trans.get(2);
//                    double rX = rot.firstAngle;
//                    double rY = rot.secondAngle;
//                    double rZ = rot.thirdAngle;


                    //offset picture from phone
                    //we need robot to picture
                }
            }

            /*else {
                telemetry.addData("VuMark", "not visible");
            }
            telemetry.update();*/

        }
    }
    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}
