package org.firstinspires.ftc.rmrobotics.opmodes.temp8121;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
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
    private Servo gemBar;
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
        gemBar = hardwareMap.servo.get("gemBar");

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
                wheelFL.getCurrentPosition(),wheelFR.getCurrentPosition(),wheelBL.getCurrentPosition(),wheelBR.getCurrentPosition());
        telemetry.update();
        VuforiaTrackable r = vuInit();
        waitForStart();
        //gemmethod
        VectorF move = vuRead(r);
        

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
                .translation(0, 106.0f * mmPerInch, 0) //temporary and arbitrary values. goal is roughly closer to the front, far right side, near the bottom ish
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZY,
                        AngleUnit.DEGREES, 0, 0, 90));
        return relicTemplate;
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
        camera2pic.rotated(AngleUnit.DEGREES, -90, 0, 1, 0); //puts the picture on the right axis

        //finds bot 2 pic and bot 2 field using the measurements from camera 2 pic
        bot2pic = camera2pic.multiplied(bot2camera);
        bot2field = bot2pic.multiplied(pic2field);
        telemetry.addData("b2f", format(bot2field));

        //setting the movement vectors for getting to the right column
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
    public VectorF getVector(OpenGLMatrix bot, float desX, float desY) {
        VectorF amHere = bot.getTranslation();
        VectorF goThere = new VectorF(0, desX, desY);
        return goThere.subtracted(amHere);
    }
    public void encoderDrive(double speed,
                             double dist,
                             double timeoutS) {
        int[] target = new int[4];
        ArrayList<int[]> pos = new ArrayList<>();
        int[] currPos = new int[4];
        double  tickRev = 1120 ;    // The number of ticks or counts in per revolution in our neverest 40 motors
        double  invGearRatio = 0.5 ;     // This is < 1.0 if geared UP, our drive train is geared 2:1
        double  diameterBoi = 101.6;     // The circumference of our mecanums in millimeters 101.6
        double  tickMM  = (tickRev * invGearRatio) /
                (diameterBoi * 3.1415);

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Turn On RUN_TO_POSITION
            wheelFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wheelFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wheelBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wheelBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Determine new target position, and pass to motor controller
            target[0] = wheelFL.getCurrentPosition() + (int)(dist * tickMM);
            target[1] = wheelFR.getCurrentPosition() + (int)(dist * tickMM);
            target[2] = wheelBL.getCurrentPosition() + (int)(dist * tickMM);
            target[3] = wheelBR.getCurrentPosition() + (int)(dist * tickMM);
            wheelFL.setTargetPosition(target[0]);
            wheelBL.setTargetPosition(target[1]);
            wheelFR.setTargetPosition(target[2]);
            wheelBR.setTargetPosition(target[3]);

            // reset the timeout time and start motion.
            runtime.reset();

            wheelFL.setPower(Math.abs(speed));
            wheelFR.setPower(Math.abs(speed));
            wheelBL.setPower(Math.abs(speed));
            wheelBR.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            telemetry.addData("Checkpoint 1.5",runtime.seconds());
            telemetry.update();
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    ((wheelFL.isBusy() && wheelBL.isBusy()) || (wheelFR.isBusy() && wheelBR.isBusy()))) {


                // Display it for the driver.
                telemetry.addData("Checkpoint 2", "Running to %d: %d: %d: %d", target[0], target[1], target[2], target[3]);
//                telemetry.addData("Running at ", wheelFL.getCurrentPosition());
//                telemetry.addData("Running at ", wheelBL.getCurrentPosition());
//                telemetry.addData("Running at ", wheelFR.getCurrentPosition());
//                telemetry.addData("Running at ", wheelBR.getCurrentPosition());
//                telemetry.addData("Runtime ",runtime.seconds());

                currPos[0] = wheelFL.getCurrentPosition();
                currPos[1] = wheelBL.getCurrentPosition();
                currPos[2] = wheelFR.getCurrentPosition();
                currPos[3] = wheelBR.getCurrentPosition();
                pos.add(currPos);
                telemetry.update();
            }

            // Stop all motion;
            wheelFL.setPower(0);
            wheelFR.setPower(0);
            wheelBL.setPower(0);
            wheelBR.setPower(0);

            telemetry.addData("Status","Checkpoint 3");
            telemetry.update();

            // Turn off RUN_TO_POSITION
            wheelFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wheelFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wheelBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wheelBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            telemetry.addData("List size: ", pos.size());
            telemetry.addData("1/5", "Running to %d: %d: %d: %d", pos.get(200)[0], pos.get(200)[1], pos.get(200)[2], pos.get(200)[3]);
            telemetry.addData("2/5", "Running to %d: %d: %d: %d", pos.get(pos.size()/4)[0], pos.get(pos.size()/4)[1], pos.get(pos.size()/4)[2], pos.get(pos.size()/4)[3]);
            telemetry.addData("3/5", "Running to %d: %d: %d: %d", pos.get(pos.size()/2)[0], pos.get(pos.size()/2)[1], pos.get(pos.size()/2)[2], pos.get(pos.size()/2)[3]);
            telemetry.addData("4/5", "Running to %d: %d: %d: %d", pos.get((pos.size()*3)/4)[0], pos.get((pos.size()*3)/4)[1], pos.get((pos.size()*3)/4)[2], pos.get((pos.size()*3)/4)[3]);
            telemetry.addData("5/5", "Running to %d: %d: %d: %d", pos.get(pos.size()-200)[0], pos.get(pos.size()-200)[1], pos.get(pos.size()-200)[2], pos.get(pos.size()-200)[3]);
            telemetry.update();

            //  sleep(250);   // optional pause after each move
        }
    }
}
