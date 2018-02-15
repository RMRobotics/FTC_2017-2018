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

import android.util.Log;
import com.kauailabs.navx.ftc.*;

/**
 * Created by riasebastian on 2/12/18.
 */

@Autonomous(name="Part2", group ="woRMholeConfig")
public class Part2 extends LinearOpMode
{

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
    private Servo armT;
    private Servo gemBarX;
    private Servo gemBarY;

    static final double  tickRev = 1120 ;    // The number of ticks or counts in per revolution in our neverest 40 motors
    static final double  invGearRatio = 0.5 ;     // This is < 1.0 if geared UP, our drive train is geared 2:1
    static final double  diameterBoi = 101.6 ;     // The circumference of our mecanums in millimeters
    static final double  tickMM  = (tickRev * invGearRatio) /
            (diameterBoi * 3.1415);
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime time = new ElapsedTime();
    VuforiaLocalizer vuforia;

    @Override public void runOpMode()
    {
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
        gemBarX = hardwareMap.servo.get("gemBar1");
        gemBarY = hardwareMap.servo.get("gemBar2");

        VectorF move = null;
        OpenGLMatrix bot2field = new OpenGLMatrix();
        float mmPerInch        = 25.4f;

        //Activate Vuforia
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AckoWtn/////AAAAGan7WAnq/0UVmQZG3sp7smBgRCNBnU1p+HmsTrC+W9TyxqaMlhFirDXglelvJCX4yBiO8oou6n7UWBfdRFbKHDqz0NIo5VcNHyhelmm0yK0vGKxoU0NZbQzjh5qVWnI/HRoFjM3JOq/LB/FTXgCcEaNGhXAqnz7nalixMeP8oRQlgX5nRVX4uE6w0K4yqIc5/FIDh1tn7PldiflmvNPhOW6FukPQD3d02wEnZB/JEchSSBzDbFA10XSgtYzXiweQI5tj+D5llLRrLh0mcWeouv55oSmya5RxUC26uEuO7bCAwyolWIuUr2Wh5oAG483nTD4vFhdjVMT7f0ovLO73C6xr2AXpNwen9IExRxBeosQ4";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");
        telemetry.update();
//        waitForStart();
        relicTrackables.activate();
        waitForStart();

        while (opModeIsActive()) {
            //Close claws

            //Lift the lift so the block doesn't drag
            lift.setPower(-0.2);
            holdUp(0.5);

            //Stop lifting
            lift.setPower(0);

            //Drop gembar
            gemBarY.setPosition(-1);

            //Detect color of jewel to the right
            boolean detected = false;
            String color = "";
            time.reset();
            while ((!detected) && (time.seconds() < 5)) {
                telemetry.addData("Red  ", colorSensor.red());
                telemetry.addData("Green", colorSensor.green());
                telemetry.addData("Blue ", colorSensor.blue());
                telemetry.update();

                if ((colorSensor.blue() > 0) && (colorSensor.red() < 1)) {
                    detected = true;
                    color = "Blue";
                } else if ((colorSensor.blue() < 1) && (colorSensor.red() > 0)) {
                    detected = true;
                    color = "Red";
                } else
                    telemetry.addData("Not yet bois", "");

                telemetry.update();
            }
            holdUp(1);
            if (color.equals("")) {
                gemBarY.setPosition(1);
            }

            //Knock off proper jewel
            if (color.equals("Red"))
                gemBarX.setPosition(-1);
            else if (color.equals("Blue"))
                gemBarX.setPosition(1);
            holdUp(1);

            //Retract
            gemBarX.setPosition(0);
            holdUp(.5);
            gemBarY.setPosition(1);
            holdUp(.5);

            //Move to read Vuforia
            encoderDrive(0.1, 3*mmPerInch,5);

            //Decode the Vuforia
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            while (vuMark == RelicRecoveryVuMark.UNKNOWN) {
                vuMark = RelicRecoveryVuMark.from(relicTemplate);
                if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                    telemetry.addData("VuMark", "%s visible", vuMark);
                    telemetry.update();
                    parameters = new VuforiaLocalizer.Parameters();
                }
            }

            //Determine the location for the cryptobox
            if (vuMark == RelicRecoveryVuMark.LEFT) {
                move = getVector(bot2field, 24 * mmPerInch, 76.37f * mmPerInch);
            }
            if (vuMark == RelicRecoveryVuMark.CENTER) {
                move = getVector(bot2field, 24 * mmPerInch, 84 * mmPerInch);
            }
            if (vuMark == RelicRecoveryVuMark.RIGHT) {
                move = getVector(bot2field, 24 * mmPerInch, 91.63f * mmPerInch);
            }

            //Moves to in front of the cryptobox
            encoderDrive(0.1, move.get(1) * mmPerInch, 10);

            //Rotate 90 degrees
            rotate90();

            //Move up 6 inches
            encoderDrive(0.1, 6 * mmPerInch, 10);

            //Release the beast
            clawTL.setPosition(-1);
            clawTR.setPosition(1);
        }
    }

    public VectorF getVector(OpenGLMatrix bot, float desX, float desY)
    {
        VectorF amHere = bot.getTranslation();
        VectorF goThere = new VectorF(0, desX ,desY);
        return goThere.subtracted(amHere);
    }

    public void rotate90()
    {

    }

    //Pauses for time num
    public void holdUp(double num)
    {
        time.reset();
        while (time.seconds() < num) {}
    }

    public void encoderDrive(double speed,
                             double dist,
                             double timeoutS) {
        int target;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            target = wheelFL.getCurrentPosition() + (int)(dist * tickMM);
            wheelFL.setTargetPosition(target);
            wheelBL.setTargetPosition(target);
            wheelFR.setTargetPosition(target);
            wheelBR.setTargetPosition(target);

            // Turn On RUN_TO_POSITION
            wheelFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wheelFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wheelBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wheelBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (wheelFL.isBusy() && wheelFR.isBusy() && wheelBL.isBusy() && wheelBR.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to ", target);
                telemetry.addData("Path2",  "Running at ", wheelFL.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            wheelFL.setPower(0);
            wheelFL.setPower(0);
            wheelFL.setPower(0);
            wheelFL.setPower(0);

            // Turn off RUN_TO_POSITION
            wheelFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wheelFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wheelBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wheelBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
}
