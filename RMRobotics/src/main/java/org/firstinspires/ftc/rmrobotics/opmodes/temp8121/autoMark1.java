package org.firstinspires.ftc.rmrobotics.opmodes.temp8121;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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
 * Created by Kameron on 10/24/2017.
 */

@Autonomous(name="autoMark1", group ="woRMholeConfig")
public class autoMark1 extends LinearOpMode{

    private DcMotor wheelFL;
    private DcMotor wheelFR;
    private DcMotor wheelBL;
    private DcMotor wheelBR;
    private DcMotor lift;
    private DcMotor arm;
    private Servo clawBL;
    private Servo clawBR;
    private Servo clawTL;
    private Servo clawTR;
    private Servo armT;
    private CRServo armB;

    private ElapsedTime time = new ElapsedTime();

    public static final String TAG = "Auto Version 1";
/*    OpenGLMatrix lastLocation = null;
   VuforiaLocalizer vuforia;*/

    @Override public void runOpMode() {


        wheelFL = hardwareMap.dcMotor.get("wheelFL");
        wheelFR = hardwareMap.dcMotor.get("wheelFR");
        wheelBL = hardwareMap.dcMotor.get("wheelBL");
        wheelBR = hardwareMap.dcMotor.get("wheelBR");
        wheelFL.setDirection(DcMotorSimple.Direction.REVERSE);
        wheelBL.setDirection(DcMotorSimple.Direction.REVERSE);
        lift = hardwareMap.dcMotor.get("lift");
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setPower(0);
        arm = hardwareMap.dcMotor.get("arm");
        clawBL = hardwareMap.servo.get("clawBL");
        clawBR = hardwareMap.servo.get("clawBR");
        armT = hardwareMap.servo.get("armT");
        armB = hardwareMap.crservo.get("armB");
        clawTL = hardwareMap.servo.get("clawTL");
        clawTR = hardwareMap.servo.get("clawTR");
        armB.setDirection(CRServo.Direction.FORWARD);
//        gemBar = hardwareMap.servo.get("gemBar");
        clawBL.setPosition(-0.7);
        clawBR.setPosition(1);
        armT.setPosition(0.5);
        armB.setPower(0);
        clawTL.setPosition(-1);
        clawTR.setPosition(1);
//        gemBar.setPosition(0);

        double timeToStance, timeToColumn, rotate90, powerino; //rotate90 is the amount of time that it takes to rotate 90 degrees

        timeToStance = 2;
        timeToColumn = 2;
        rotate90 = 0.7;
       /*
       RelicRecoveryVuMark column = runVuforia();
*/
        time.reset();
/*
       move(rotate90, 1.0, 0.0, 1.0);
*/



        //move(timeToStance, 0.05, 0.0, 0.0);
        move(rotate90, 0.05, 0.0, 90.0); //if left turn

/*        if (column == RelicRecoveryVuMark.LEFT)
           timeToColumn = 0;
       if (column == RelicRecoveryVuMark.CENTER)
           timeToColumn = 0;
       if (column == RelicRecoveryVuMark.RIGHT)
           timeToColumn = 0;*/
        move(timeToColumn, 0.05, 0.0, 0.0);



    }


    public void move(double duration, double power, double angle, double rotate) {
        angle *= (Math.PI / 180);
        time.reset();
        while (time.seconds() < duration) {
            wheelFL.setPower(power * Math.sin(angle + (Math.PI / 4)) + rotate);
            wheelFR.setPower(power * Math.cos(angle + (Math.PI / 4)) - rotate);
            wheelBL.setPower(power * Math.cos(angle + (Math.PI / 4)) + rotate);
            wheelBR.setPower(power * Math.sin(angle + (Math.PI / 4)) - rotate);
        }
    }


/*    public RelicRecoveryVuMark runVuforia() {
       int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
       VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
       parameters.vuforiaLicenseKey = "ATsODcD/////AAAAAVw2lR...d45oGpdljdOh5LuFB9nDNfckoxb8COxKSFX";
       parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
       this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
       VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
       VuforiaTrackable relicTemplate = relicTrackables.get(0);
       relicTemplate.setName("relicVuMarkTemplate");
       telemetry.addData(">", "Press Play to start");
       telemetry.update();
       waitForStart();
       relicTrackables.activate();
       while (opModeIsActive()) {
           RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
           if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
               telemetry.addData("VuMark", "%s visible", vuMark);
               OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();
//                telemetry.addData("Pose", format(pose));
//                if (pose != null) {
//                    VectorF trans = pose.getTranslation();
//                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
//
//                    double tX = trans.get(0);
//                    double tY = trans.get(1);
//                    double tZ = trans.get(2);
//
//                    double rX = rot.firstAngle;
//                    double rY = rot.secondAngle;
//                    double rZ = rot.thirdAngle;
//                }
//            }
//            else {
//                telemetry.addData("VuMark", "not visible");
//            }
               telemetry.update();
               return vuMark;
           }
       }
   }*/


}