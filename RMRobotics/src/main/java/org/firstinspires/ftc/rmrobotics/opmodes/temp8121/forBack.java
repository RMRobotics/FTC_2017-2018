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

@Autonomous(name="forBack", group ="woRMholeConfig")
public class forBack extends LinearOpMode {

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
    /*    OpenGLMatrix lastLocation = null;*/
    VuforiaLocalizer vuforia;

    @Override
    public void runOpMode() {


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
        clawTR.setPosition(1);
        clawTL.setPosition(-1);
        armT.setPosition(0.5);
        armB.setPower(0);
        clawBL.setPosition(-0.7);
        clawBR.setPosition(1);
//        gemBar.setPosition(0);

        double timeToStance, timeToColumn, rotate90, powerino; //rotate90 is the amount of time that it takes to rotate 90 degrees


        timeToStance = 2;
        timeToColumn = 0.75;
        rotate90 = 0.87;//0.82

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AckoWtn/////AAAAGan7WAnq/0UVmQZG3sp7smBgRCNBnU1p+HmsTrC+W9TyxqaMlhFirDXglelvJCX4yBiO8oou6n7UWBfdRFbKHDqz0NIo5VcNHyhelmm0yK0vGKxoU0NZbQzjh5qVWnI/HRoFjM3JOq/LB/FTXgCcEaNGhXAqnz7nalixMeP8oRQlgX5nRVX4uE6w0K4yqIc5/FIDh1tn7PldiflmvNPhOW6FukPQD3d02wEnZB/JEchSSBzDbFA10XSgtYzXiweQI5tj+D5llLRrLh0mcWeouv55oSmya5RxUC26uEuO7bCAwyolWIuUr2Wh5oAG483nTD4vFhdjVMT7f0ovLO73C6xr2AXpNwen9IExRxBeosQ4";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");
        telemetry.update();
        waitForStart();
        relicTrackables.activate();

        waitForStart();

        while(opModeIsActive()) {


            move(0.9, 0.5, 0.0, 0.0);
            move(0.9, -0.5, 0.0, 0.0);

            wheelBL.setPower(0);
            wheelBR.setPower(0);
            wheelFL.setPower(0);
            wheelFR.setPower(0);

//            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
//            while (vuMark == RelicRecoveryVuMark.UNKNOWN){
//                vuMark = RelicRecoveryVuMark.from(relicTemplate);
//                if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
//                    telemetry.addData("VuMark", "%s visible", vuMark);
//                    telemetry.update();
//                }
//            }
//
//            if (vuMark.equals(RelicRecoveryVuMark.LEFT))
//            {
//                move(0.90, 0.5, 0.0, 0.0);
//            }
//
//            if (vuMark.equals(RelicRecoveryVuMark.CENTER))
//            {
//                move(0.60, 0.5, 0.0, 0.0);
//            }
//
//            if (vuMark.equals(RelicRecoveryVuMark.RIGHT))
//            {
//                move(0.4, 0.5, 0.0, 0.0);
//            }
//
//            move(rotate90, 0.05, 0.0, 90.0);
//
//            move(0.5, 0.5, 0, 0);
//
//            clawTR.setPosition(1);
//            clawTL.setPosition(-1);
//            clawBL.setPosition(-0.7);
//            clawBR.setPosition(1);
//
//            move(0.2, -0.5, 0, 0);


            break;
        }
        /*
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AckoWtn/////AAAAGan7WAnq/0UVmQZG3sp7smBgRCNBnU1p+HmsTrC+W9TyxqaMlhFirDXglelvJCX4yBiO8oou6n7UWBfdRFbKHDqz0NIo5VcNHyhelmm0yK0vGKxoU0NZbQzjh5qVWnI/HRoFjM3JOq/LB/FTXgCcEaNGhXAqnz7nalixMeP8oRQlgX5nRVX4uE6w0K4yqIc5/FIDh1tn7PldiflmvNPhOW6FukPQD3d02wEnZB/JEchSSBzDbFA10XSgtYzXiweQI5tj+D5llLRrLh0mcWeouv55oSmya5RxUC26uEuO7bCAwyolWIuUr2Wh5oAG483nTD4vFhdjVMT7f0ovLO73C6xr2AXpNwen9IExRxBeosQ4";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");
        telemetry.update();
        waitForStart();
        relicTrackables.activate();
        while (opModeIsActive()) {
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                telemetry.addData("VuMark", "%s visible", vuMark);
                telemetry.update();
            }

            if (vuMark.equals("LEFT")) {
                move(0.75, 0.5, 0.0, 0.0);
                telemetry.addData("ur mum gey:", "%s VuMark:", vuMark);
                telemetry.update();
            }
            if (vuMark.equals(RelicRecoveryVuMark.LEFT))
            {
                move(0.75, 0.5, 0.0, 0.0);
            }
            if (vuMark.equals("RIGHT")) {
                move(1.5, 0.5, 0.0, 0.0);
            }
            if (vuMark.equals("CENTER")) {
                move(rotate90, 0.05, 0.0, 90.0);
            }
        }*/


        //move(timeToStance, 0.05, 0.0, 0.0);

        //move(timeToColumn, 0.5, 0.0, 0.0);

        //move(rotate90, 0.05, 0.0, 90.0); //if left turn

/*        if (column == RelicRecoveryVuMark.LEFT)
            timeToColumn = 0;
        if (column == RelicRecoveryVuMark.CENTER)
            timeToColumn = 0;
        if (column == RelicRecoveryVuMark.RIGHT)
            timeToColumn = 0;*/
        //move(timeToColumn, 0.5, 45.0, 0.0);


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
}




