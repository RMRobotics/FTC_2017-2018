package org.firstinspires.ftc.rmrobotics.opmodes.temp8121;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
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

import java.util.ArrayList;

import static com.sun.tools.javac.util.Constants.format;

/**
 * Created by rotom on 10/17/2017.
 */

@Autonomous(name="testVuAutoP1", group ="woRMholeConfig")
public class woRMhole_autoP1 extends LinearOpMode {
    private ColorSensor colorSensor;
    private DcMotor wheelFL;
    private DcMotor wheelFR;
    private DcMotor wheelBL;
    private DcMotor wheelBR;
    private DcMotor lift;
    private Servo clawBL;
    private Servo clawBR;
    private Servo clawTL;
    private Servo clawTR;
    private Servo gemBar1;
    private Servo gemBar2;
    private ElapsedTime runtime = new ElapsedTime();
    public static final String TAG = "Vuforia VuMark Sample";
    //pic2field*camera2pic=camera2field*bot2camera=bot2field
    private OpenGLMatrix bot2camera, camera2pic, pic2field, bot2pic, bot2field;
    VuforiaLocalizer vuforia;

    @Override
    public void runOpMode() {
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");
        wheelFL = hardwareMap.dcMotor.get("wheelFL");
        wheelFR = hardwareMap.dcMotor.get("wheelFR");
        wheelBL = hardwareMap.dcMotor.get("wheelBL");
        wheelBR = hardwareMap.dcMotor.get("wheelBR");
        lift = hardwareMap.dcMotor.get("lift");
        clawBL = hardwareMap.servo.get("clawBL");
        clawBR = hardwareMap.servo.get("clawBR");
        clawTL = hardwareMap.servo.get("clawTL");
        clawTR = hardwareMap.servo.get("clawTR");
        gemBar1 = hardwareMap.servo.get("gemBar1");
        gemBar2 = hardwareMap.servo.get("gemBar2");
        clawBL = hardwareMap.servo.get("clawBL");
        clawBR = hardwareMap.servo.get("clawBR");
        clawTL = hardwareMap.servo.get("clawTL");
        clawTR = hardwareMap.servo.get("clawTR");

        wheelFL.setDirection(DcMotorSimple.Direction.REVERSE);
        wheelBL.setDirection(DcMotorSimple.Direction.REVERSE);
        clawBL.setPosition(0.2);
        clawBR.setPosition(0.5);
        clawTR.setPosition(0.7);
        clawTL.setPosition(0.35);
        gemBar1.setPosition(1);
        gemBar2.setPosition(0.4);
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");
        colorSensor.enableLed(true);

        //encoder setup and reset
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();
        wheelBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d :%7d :%7d",
                wheelFL.getCurrentPosition(), wheelFR.getCurrentPosition(), wheelBL.getCurrentPosition(), wheelBR.getCurrentPosition());
        telemetry.update();

        //Actual stuff that happens and is awesome and dope and cool and yeet
        VuforiaTrackable r = vuInit();
        waitForStart();
        while (opModeIsActive()) {
            lift.setPower(0.2);
            holdUp(0.5);
            lift.setPower(0);
            holdUp(0.5);
            gemKnock();
            VectorF move = vuRead(r);
            encoderDrive(0.5, move.get(2), 10);
            holdUp(1);
            deadR(0.4, 0.05, 0.0, 90.0);
            holdUp(1);
            encoderDrive(0.5, move.get(1), 10);
            holdUp(1);
            clawTR.setPosition(0.7);
            clawTL.setPosition(0.35);
            clawBL.setPosition(0.2);
            clawBR.setPosition(0.85);
        }
    }
    public VuforiaTrackable vuInit() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AckoWtn/////AAAAGan7WAnq/0UVmQZG3sp7smBgRCNBnU1p+HmsTrC+W9TyxqaMlhFirDXglelvJCX4yBiO8oou6n7UWBfdRFbKHDqz0NIo5VcNHyhelmm0yK0vGKxoU0NZbQzjh5qVWnI/HRoFjM3JOq/LB/FTXgCcEaNGhXAqnz7nalixMeP8oRQlgX5nRVX4uE6w0K4yqIc5/FIDh1tn7PldiflmvNPhOW6FukPQD3d02wEnZB/JEchSSBzDbFA10XSgtYzXiweQI5tj+D5llLRrLh0mcWeouv55oSmya5RxUC26uEuO7bCAwyolWIuUr2Wh5oAG483nTD4vFhdjVMT7f0ovLO73C6xr2AXpNwen9IExRxBeosQ4";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        telemetry.update();
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");
        relicTrackables.activate();

        //bot2camera and pic2field are constants
        float mmPerInch = 25.4f;
        float mmBotWidth = 18 * mmPerInch;
        float mmFTCFieldWidth = (12 * 12 - 2) * mmPerInch;
        bot2camera = OpenGLMatrix
                .translation(3.75f * mmPerInch, 5.37f * mmPerInch, 6.248f * mmPerInch) //temporary and arbitrary values. goal is roughly closer to the front, far right side, near the bottom ish
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZY,
                        AngleUnit.DEGREES, 0, 0, 0));

        //trans: bottom left of field is 0,0. red bottom picture is (0,34,0) in inches. blue bottom (144,34,0) red top (0,106,0) blue top (144,106,0)
        //rot: (0,0,90) (0,0,-90), (0,0,0), (0,0,0)
        //this program is testing red top
        pic2field = OpenGLMatrix
                .translation(0, 111.5f * mmPerInch, 5.75f*mmPerInch) //temporary and arbitrary values. goal is roughly closer to the front, far right side, near the bottom ish
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZY,
                        AngleUnit.DEGREES, 0, 0, 90));
        return relicTemplate;
    }
    public void gemKnock() {
        boolean detected = false;
        String color = "";

        runtime.reset();
        gemBar1.setPosition(-1);
        holdUp(1.5);
        while ((!detected) && (runtime.seconds() < 5)) {
            if ((colorSensor.blue() > 0) && (colorSensor.red() < 1)) {
                detected = true;
                color = "Blue";
            } else if ((colorSensor.blue() < 1) && (colorSensor.red() > 0)) {
                detected = true;
                color = "Red";
            } else
                telemetry.addData("Not yet bois", "");
        }
        holdUp(1);
        if (color.equals("")){
            gemBar1.setPosition(0);
        }
        else if (color.equals("Red"))
        {
            gemBar2.setPosition(-0.5);
            holdUp(1);
            gemBar2.setPosition(0.4);
            holdUp(1);
            gemBar1.setPosition(1);
        }
        else {
            gemBar2.setPosition(0.8);
            holdUp(1);
            gemBar2.setPosition(0.4);
            holdUp(1);
            gemBar1.setPosition(1);
        }
    }
    public VectorF vuRead(VuforiaTrackable relicTemplate) {
        VectorF move = new VectorF(0, 0, 0);
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.UNKNOWN;

        while (vuMark == RelicRecoveryVuMark.UNKNOWN) {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                telemetry.addData("VuMark", "%s visible", vuMark);
                //camera2pic is the raw vuforia reading
                camera2pic = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose(); //creates an openGLMatrix for the position of the picture relative to the phone
            } else {
                telemetry.addData("VuMark", "not visible");
            }
        }
//        camera2pic.rotated(AngleUnit.DEGREES, -90, 0, 1, 0); //puts the picture on the right axis
        //finds bot 2 pic and bot 2 field using the measurements from camera 2 pic
        bot2pic = camera2pic.multiplied(bot2camera);
        bot2field = bot2pic.multiplied(pic2field);
        telemetry.addData("b2f", format(bot2field));
        if (vuMark == RelicRecoveryVuMark.LEFT) {
            move = getVector(bot2field, 609.6f, 1939.798f);
        }
        if (vuMark == RelicRecoveryVuMark.CENTER) {
            move = getVector(bot2field, 609.6f, 2133.6f);
        }
        if (vuMark == RelicRecoveryVuMark.RIGHT) {
            move = getVector(bot2field, 609.6f, 2327.402f);
        }
        telemetry.update();
        return move;
    }
    public void encoderDrive(double speed, double dist, double timeoutS) {
        int[] target = new int[4];
        ArrayList<int[]> pos = new ArrayList<>();
        int[] currPos = new int[4];
        double  tickRev = 1120 ;    // The number of ticks or counts in per revolution in our neverest 40 motors
        double  invGearRatio = 0.5 ;     // This is < 1.0 if geared UP
        double  diameterBoi = 101.6;     // The circumference of our mecanums in millimeters 101.6
        double  tickMM  = (tickRev * invGearRatio) /
                (diameterBoi * 3.1415);

        wheelFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheelFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheelBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheelBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        target[0] = wheelFL.getCurrentPosition() + (int)(dist * tickMM);
        target[1] = wheelFR.getCurrentPosition() + (int)(dist * tickMM);
        target[2] = wheelBL.getCurrentPosition() + (int)(dist * tickMM);
        target[3] = wheelBR.getCurrentPosition() + (int)(dist * tickMM);
        wheelFL.setTargetPosition(target[0]);
        wheelBL.setTargetPosition(target[1]);
        wheelFR.setTargetPosition(target[2]);
        wheelBR.setTargetPosition(target[3]);
        runtime.reset();
        wheelFL.setPower(Math.abs(speed));
        wheelFR.setPower(Math.abs(speed));
        wheelBL.setPower(Math.abs(speed));
        wheelBR.setPower(Math.abs(speed));

        while ((runtime.seconds() < timeoutS) && ((wheelFL.isBusy() && wheelBL.isBusy()) || (wheelFR.isBusy() && wheelBR.isBusy()))) {
            currPos[0] = wheelFL.getCurrentPosition();
            currPos[1] = wheelBL.getCurrentPosition();
            currPos[2] = wheelFR.getCurrentPosition();
            currPos[3] = wheelBR.getCurrentPosition();
            pos.add(currPos);
        }
        wheelFL.setPower(0);
        wheelFR.setPower(0);
        wheelBL.setPower(0);
        wheelBR.setPower(0);
        wheelFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            telemetry.addData("List size: ", pos.size());
//            telemetry.addData("1/5", "Running to %d: %d: %d: %d", pos.get(200)[0], pos.get(200)[1], pos.get(200)[2], pos.get(200)[3]);
//            telemetry.addData("2/5", "Running to %d: %d: %d: %d", pos.get(pos.size()/4)[0], pos.get(pos.size()/4)[1], pos.get(pos.size()/4)[2], pos.get(pos.size()/4)[3]);
//            telemetry.addData("3/5", "Running to %d: %d: %d: %d", pos.get(pos.size()/2)[0], pos.get(pos.size()/2)[1], pos.get(pos.size()/2)[2], pos.get(pos.size()/2)[3]);
//            telemetry.addData("4/5", "Running to %d: %d: %d: %d", pos.get((pos.size()*3)/4)[0], pos.get((pos.size()*3)/4)[1], pos.get((pos.size()*3)/4)[2], pos.get((pos.size()*3)/4)[3]);
//            telemetry.addData("5/5", "Running to %d: %d: %d: %d", pos.get(pos.size()-200)[0], pos.get(pos.size()-200)[1], pos.get(pos.size()-200)[2], pos.get(pos.size()-200)[3]);
//            telemetry.update();
        }
    public void deadR(double duration, double power, double angle, double rotate) {
        angle *= (Math.PI / 180);
        runtime.reset();
        while (runtime.seconds() < duration) {
            wheelFL.setPower(power * Math.sin(angle + (Math.PI / 4)) + rotate);
            wheelFR.setPower(power * Math.cos(angle + (Math.PI / 4)) - rotate);
            wheelBL.setPower(power * Math.cos(angle + (Math.PI / 4)) + rotate);
            wheelBR.setPower(power * Math.sin(angle + (Math.PI / 4)) - rotate);
        }
    }
    public VectorF getVector(OpenGLMatrix bot, float desX, float desY) {
        VectorF amHere = bot.getTranslation();
        VectorF goThere = new VectorF(0, desX, desY);
        return goThere.subtracted(amHere);
    }
    public void holdUp(double num)
    {
        runtime.reset();
        while (runtime.seconds() < num) {}
    }

}
