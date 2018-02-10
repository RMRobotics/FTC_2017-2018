package org.firstinspires.ftc.rmrobotics.opmodes.temp8121;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

/**
 * Created by rotom on 10/17/2017.
 */

@Autonomous(name="testEncoders", group ="encoderFig")
public class testEncoders extends LinearOpMode{

    //hardware declarations
    private DcMotor wheelFL;
    private DcMotor wheelFR;
    private DcMotor wheelBL;
    private DcMotor wheelBR;

    //Set up for encoders
    static final double  tickRev = 1120 ;    // The number of ticks or counts in per revolution in our neverest 40 motors
    static final double  invGearRatio = 0.5 ;     // This is < 1.0 if geared UP, our drive train is geared 2:1
    static final double  diameterBoi = 101.6 ;     // The circumference of our mecanums in millimeters
    static final double  tickMM  = (tickRev * invGearRatio) /
            (diameterBoi * 3.1415);
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime time = new ElapsedTime();


    @Override public void runOpMode() {

        //hardware map in wormhole config in the UTIL folder
        wheelFL = hardwareMap.dcMotor.get("wheelFL");
        wheelFR = hardwareMap.dcMotor.get("wheelFR");
        wheelBL = hardwareMap.dcMotor.get("wheelBL");
        wheelBR = hardwareMap.dcMotor.get("wheelBR");
        wheelFL.setDirection(DcMotorSimple.Direction.REVERSE);
        wheelBL.setDirection(DcMotorSimple.Direction.REVERSE);

        wheelFL.setPower(0.5);
        wheelBL.setPower(0.5);
        wheelFR.setPower(0.5);
        wheelBR.setPower(0.5);

        holdUp(0.5);

        wheelFL.setPower(0);
        wheelBL.setPower(0);
        wheelFR.setPower(0);
        wheelBR.setPower(0);

        //encoder setup and reset
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();
        wheelBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //values used for vuforia navigation
        float mmPerInch        = 25.4f;
        float mmBotWidth       = 18 * mmPerInch;            // ... or whatever is right for your robot
        float mmFTCFieldWidth  = (12*12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels

        //actual actions performed in the autonomous
        telemetry.addData("Status", "Checkpoint 1");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {


            wheelFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wheelFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wheelBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wheelBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Send telemetry message to indicate successful Encoder reset
            //telemetry.addData("Path0",  "Starting at %7d :%7d",
            //        wheelFL.getCurrentPosition());
            //telemetry.update();


            // Step through each leg of the path,
            // Note: Reverse movement is obtained by setting a negative distance (not speed)
            encoderDrive(0.3,6,5.0);  // S1: Forward 47 Inches with 5 Sec timeout
            stop();
        }
    }
    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

    public void holdUp(double num)
    {
        time.reset();
        while (time.seconds() < num) {}
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

            // Turn On RUN_TO_POSITION
            wheelFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wheelFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wheelBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wheelBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Determine new target position, and pass to motor controller
            target = wheelFL.getCurrentPosition() + (int)(dist * tickMM);
            wheelFL.setTargetPosition(target);
            target = wheelBL.getCurrentPosition() + (int)(dist * tickMM);
            wheelBL.setTargetPosition(target);
            target = wheelFR.getCurrentPosition() + (int)(dist * tickMM);
            wheelFR.setTargetPosition(target);
            target = wheelBR.getCurrentPosition() + (int)(dist * tickMM);
            wheelBR.setTargetPosition(target);

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
                    (wheelFL.isBusy() || wheelFR.isBusy() || wheelBL.isBusy() || wheelBR.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to ", target);
                telemetry.addData("Path2",  "Running at ", wheelFL.getCurrentPosition());
                telemetry.addData("Checkpoint 2",runtime.seconds());
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

            //  sleep(250);   // optional pause after each move
        }
    }

}
