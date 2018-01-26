package org.firstinspires.ftc.rmrobotics.opmodes.temp8121;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

@Autonomous(name="twistyForwardBoi", group ="woRMholeConfig")
@Disabled
public class twistyForwardBoi extends LinearOpMode {

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

        double time1square, rotate90; //rotate90 is the amount of time that it takes to rotate 90 degrees


        time1square = 0.9;
        rotate90 = 0.95;//0.82

        waitForStart();

        waitForStart();

        while(opModeIsActive()) {

            move(time1square, 0.5, 0.0, 0.0);
            wheelBL.setPower(0);
            wheelBR.setPower(0);
            wheelFL.setPower(0);
            wheelFR.setPower(0);

            move(rotate90, 0.05, 0.0, 90.0);

            wheelBL.setPower(0);
            wheelBR.setPower(0);
            wheelFL.setPower(0);
            wheelFR.setPower(0);


            break;
        }

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




