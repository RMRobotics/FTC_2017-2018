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
import java.text.DecimalFormat;

/**
 * Created by rotom on 10/17/2017.
 */

@Autonomous(name="testVuAuto", group ="woRMholeConfig")
public class testVuAuto extends LinearOpMode{

    //initialization of navx stuff
    private final int NAVX_DIM_I2C_PORT = 0; //port on the dim
    private AHRS navx_device;
    private navXPIDController yawPIDController;

    //hardware declarations
    private ColorSensor colorSensor;
    private DcMotor wheelFL;
    private DcMotor wheelFR;
    private DcMotor wheelBL;
    private DcMotor wheelBR;
    private DcMotor lift;
    //    private DcMotor arm;
    private Servo clawBL;
    private Servo clawBR;
    private Servo clawTL;
    private Servo clawTR;
    //    private Servo armT;
    private Servo gemBar;
    //    private CRServo armB;

    //Set up for encoders
    static final double  tickRev = 1120 ;    // The number of ticks or counts in per revolution in our neverest 40 motors
    static final double  invGearRatio = 0.5 ;     // This is < 1.0 if geared UP, our drive train is geared 2:1
    static final double  diameterBoi = 101.6 ;     // The circumference of our mecanums in millimeters
    static final double  tickMM  = (tickRev * invGearRatio) /
            (diameterBoi * 3.1415);
    private ElapsedTime runtime = new ElapsedTime();

    //vuforia initialization
    public static final String TAG = "Vuforia VuMark Sample";
    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;

    @Override public void runOpMode() {

        //hardware map in wormhole config in the UTIL folder
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

        //starting vuforia
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

        //values used for vuforia navigation
        float mmPerInch        = 25.4f;
        float mmBotWidth       = 18 * mmPerInch;            // ... or whatever is right for your robot
        float mmFTCFieldWidth  = (12*12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels



        //0,0,0 is front left bottom corner. phone is a matrix of where the phone is relative to the robot
        OpenGLMatrix camera2bot = OpenGLMatrix
                .translation(3.75f*mmPerInch, 5.37f*mmPerInch, 6.248f*mmPerInch) //temporary and arbitrary values. goal is roughly closer to the front, far right side, near the bottom ish
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZY,
                        AngleUnit.DEGREES, 0, 0, 0));
        //theoretically should have phone upright, screen facing out towards the right
        //RobotLog.ii(TAG, "phone=%s", format(phoneLocationOnRobot));

        //trans: bottom left of field is 0,0. red bottom picture is (0,34,0) in inches. blue bottom (144,34,0) red top (0,106,0) blue top (144,106,0)
        //rot: (0,0,90) (0,0,-90), (0,0,0), (0,0,0)
        //this program is testing red top
        OpenGLMatrix pic2field = OpenGLMatrix
                .translation(0,106.0f*mmPerInch,0) //temporary and arbitrary values. goal is roughly closer to the front, far right side, near the bottom ish
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZY,
                        AngleUnit.DEGREES, 0, 0, 90));

        //actual actions performed in the autonomous
        waitForStart();
        while (opModeIsActive()) {

            //gemBar

            //gyro alignment

            //loops until the camera can find the vuforia image
            OpenGLMatrix camera2pic = null;
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.UNKNOWN;
            while(vuMark == RelicRecoveryVuMark.UNKNOWN) {
                vuMark = RelicRecoveryVuMark.from(relicTemplate);
                if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                    telemetry.addData("VuMark", "%s visible", vuMark);
                    camera2pic = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose(); //creates an openGLMatrix for the position of the picture relative to the phone
                } else {
                    telemetry.addData("VuMark", "not visible");
                }
            }

            //pic2phone.translation(0.0f, 0.0f, -90.0f);
            camera2pic.rotated(AngleUnit.DEGREES, -90, 0, 1, 0); //puts the picture on the right axis

            //finds bot 2 pic and bot 2 field using the measurements from camera 2 pic
            OpenGLMatrix bot2pic = camera2pic.multiplied(camera2bot);
            OpenGLMatrix bot2field = new OpenGLMatrix();
            bot2field = bot2pic.multiplied(pic2field);
            telemetry.addData("c2p", format(camera2pic));
            telemetry.addData("b2p", format(bot2pic));
            telemetry.addData("b2f", format(bot2field));

            //setting the movement vectors for getting to the right column
            if (vuMark == RelicRecoveryVuMark.LEFT) {
                VectorF move = getVector(bot2field, 609.6f, 1939.798f);
            }
            if (vuMark == RelicRecoveryVuMark.CENTER) {
                VectorF move = getVector(bot2field, 609.6f, 2133.6f);
            }
            if (vuMark == RelicRecoveryVuMark.RIGHT) {
                VectorF move = getVector(bot2field, 609.6f, 2327.402f);
            }


//                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
//                    double tX = trans.get(0);
//                    double tY = trans.get(1);
//                    double tZ = trans.get(2);
//                    double rX = rot.firstAngle;
//                    double rY = rot.secondAngle;
//                    double rZ = rot.thirdAngle;


                    //offset picture from phone
                    //we need robot to picture

            telemetry.update();

            wheelFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wheelFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wheelBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wheelBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Send telemetry message to indicate successful Encoder reset
            telemetry.addData("Path0",  "Starting at %7d :%7d",
                    wheelFL.getCurrentPosition());
            telemetry.update();


            // Step through each leg of the path,
            // Note: Reverse movement is obtained by setting a negative distance (not speed)
            encoderDrive(0.5,48,5.0);  // S1: Forward 47 Inches with 5 Sec timeout
        }
    }
    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
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

    public VectorF getVector(OpenGLMatrix bot, float desX, float desY)
    {
        VectorF amHere = bot.getTranslation();
        VectorF goThere = new VectorF(0, desX ,desY);
        return goThere.subtracted(amHere);
    }

    public void rotateToAngle (double angle) throws InterruptedException  {

        double TARGET_ANGLE_DEGREES = angle;
        double TOLERANCE_DEGREES = 2.0;
        double MIN_MOTOR_OUTPUT_VALUE = -1.0;
        double MAX_MOTOR_OUTPUT_VALUE = 1.0;
        double YAW_PID_P = 0.005;
        double YAW_PID_I = 0.0;
        double YAW_PID_D = 0.0;


        navx_device = AHRS.getInstance(hardwareMap.deviceInterfaceModule.get("dim"),
                NAVX_DIM_I2C_PORT,
                AHRS.DeviceDataType.kProcessedData);

        /* If possible, use encoders when driving, as it results in more */
        /* predicatable drive system response.                           */


        /* Create a PID Controller which uses the Yaw Angle as input. */
        yawPIDController = new navXPIDController( navx_device,
                navXPIDController.navXTimestampedDataSource.YAW);

        /* Configure the PID controller */
        yawPIDController.setSetpoint(TARGET_ANGLE_DEGREES);
        yawPIDController.setContinuous(true);
        yawPIDController.setOutputRange(MIN_MOTOR_OUTPUT_VALUE, MAX_MOTOR_OUTPUT_VALUE);
        yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, TOLERANCE_DEGREES);
        yawPIDController.setPID(YAW_PID_P, YAW_PID_I, YAW_PID_D);
        yawPIDController.enable(true);

    /* Wait for new Yaw PID output values, then update the motors
       with the new PID value with each new output value.
     */



        runtime.reset();

        double TOTAL_RUN_TIME_SECONDS = 30.0;
        int timeout = 1000;
        navXPIDController.PIDResult yawPIDResult = new navXPIDController.PIDResult();

        while ( runtime.seconds() < TOTAL_RUN_TIME_SECONDS ) {

            if ( yawPIDController.waitForNewUpdate(yawPIDResult, timeout)) {
                if ( yawPIDResult.isOnTarget() ) {
                    wheelFL.setPower(0);
                    wheelFL.setPower(0);
                    wheelFL.setPower(0);
                    wheelFL.setPower(0);
                } else {
                    double output = yawPIDResult.getOutput();
                    if ( output < 0 ) {
                /* Rotate Left */
                        wheelFL.setPower(-output);
                        wheelFR.setPower(output);
                        wheelBL.setPower(-output);
                        wheelBR.setPower(output);
                    } else {
                /* Rotate Right */
                        wheelFL.setPower(output);
                        wheelFR.setPower(-output);
                        wheelBL.setPower(output);
                        wheelBR.setPower(-output);
                    }
                }
            }
            else {
              /* A timeout occurred */
                Log.w("navXRotateToAnglePIDOp", "Yaw PID waitForNewUpdate() TIMEOUT.");
            }
        }
    }

    public void straightLineDrive (int times, double power) throws InterruptedException {

        runtime.reset();
        byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;
        double TARGET_ANGLE_DEGREES = 90.0;
        double TOLERANCE_DEGREES = 2.0;
        double MIN_MOTOR_OUTPUT_VALUE = -1.0;
        double MAX_MOTOR_OUTPUT_VALUE = 1.0;
        double YAW_PID_P = 0.005;
        double YAW_PID_I = 0.0;
        double YAW_PID_D = 0.0;

        navx_device = AHRS.getInstance(hardwareMap.deviceInterfaceModule.get("dim"),
                NAVX_DIM_I2C_PORT,
                AHRS.DeviceDataType.kProcessedData,
                NAVX_DEVICE_UPDATE_RATE_HZ);


        /* Create a PID Controller which uses the Yaw Angle as input. */
        yawPIDController = new navXPIDController( navx_device,
                navXPIDController.navXTimestampedDataSource.YAW);

        /* Configure the PID controller */
        yawPIDController.setSetpoint(TARGET_ANGLE_DEGREES);
        yawPIDController.setContinuous(true);
        yawPIDController.setOutputRange(MIN_MOTOR_OUTPUT_VALUE, MAX_MOTOR_OUTPUT_VALUE);
        yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, TOLERANCE_DEGREES);
        yawPIDController.setPID(YAW_PID_P, YAW_PID_I, YAW_PID_D);
        yawPIDController.enable(true);

        /* reset the navX-Model device yaw angle so that whatever direction */
        /* it is currently pointing will be zero degrees.                   */

        navx_device.zeroYaw();

        /* Wait for new Yaw PID output values, then update the motors
           with the new PID value with each new output value.
         */

        final double TOTAL_RUN_TIME_SECONDS = times;
        int DEVICE_TIMEOUT_MS = 500;
        navXPIDController.PIDResult yawPIDResult = new navXPIDController.PIDResult();

        /* Drive straight forward at 1/2 of full drive speed */
        double drive_speed = power;

        while ( runtime.seconds() < TOTAL_RUN_TIME_SECONDS ) {
            if ( yawPIDController.waitForNewUpdate(yawPIDResult, DEVICE_TIMEOUT_MS) ) {
                if ( yawPIDResult.isOnTarget() ) {
                    wheelFR.setPower(drive_speed);
                    wheelFL.setPower(drive_speed);
                    wheelBR.setPower(drive_speed);
                    wheelBL.setPower(drive_speed);
                } else {
                    double output = yawPIDResult.getOutput();
                    if ( output < 0 ) {
                        /* Rotate Left */
                        wheelFL.setPower(drive_speed - output);
                        wheelBL.setPower(drive_speed - output);
                        wheelBR.setPower(drive_speed + output);
                        wheelFR.setPower(drive_speed + output);
                    } else {
                        /* Rotate Right */
                        wheelFL.setPower(drive_speed + output);
                        wheelBL.setPower(drive_speed + output);
                        wheelBR.setPower(drive_speed - output);
                        wheelFR.setPower(drive_speed - output);
                    }
                }
            } else {
			    /* A timeout occurred */
                Log.w("navXRotateToAnglePIDOp", "Yaw PID waitForNewUpdate() TIMEOUT.");
            }
        }
    }
}


