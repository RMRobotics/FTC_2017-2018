package org.firstinspires.ftc.rmrobotics.opmodes.temp8121.tests;

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

import java.util.ArrayList;

/**
 * Created by rotom on 10/17/2017.
 */

@Autonomous(name="testEncoders69", group ="encoderFig")
public class testEncoders2 extends LinearOpMode{

    //hardware declarations
    private DcMotor wheelFL;
    private DcMotor wheelFR;
    private DcMotor wheelBL;
    private DcMotor wheelBR;

    //Set up for encoders
    static final double  tickRev = 1120 ;    // The number of ticks or counts in per revolution in our neverest 40 motors
    static final double  invGearRatio = 0.5 ;     // This is < 1.0 if geared UP, our drive train is geared 2:1
    static final double  diameterBoi = 101.6;     // The circumference of our mecanums in millimeters 101.6
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

        //encoder setup and reset
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();
        wheelBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
            encoderDrive(0.3, 6 * mmPerInch, 10);
            holdUp(8);
            encoderDrive(0.3, 12 * mmPerInch,10);  // S1: Forward 47 Inches with 5 Sec timeout
            holdUp(8);
            encoderDrive(0.3, 18 * mmPerInch, 10);
            holdUp(8);
//            encoderTurn(0.3, 24*mmPerInch,10);
//            holdUp(8);
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
        int[] target = new int[4];
        ArrayList<int[]> pos = new ArrayList<>();
        int[] currPos = new int[4];

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
    public void encoderTurn(double speed,
                            double dist,
                            double timeoutS) {
        int[] target = new int[4];
        ArrayList<int[]> pos = new ArrayList<>();
        int[] currPos = new int[4];

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Turn On RUN_TO_POSITION
            wheelFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wheelFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wheelBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wheelBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Determine new target position, and pass to motor controller
            target[0] = wheelFL.getCurrentPosition() + (int)((dist+((dist-6)/3)) * tickMM);
            target[1] = wheelFR.getCurrentPosition() + (int)((dist+((dist-6)/3)) * tickMM);
            target[2] = wheelBL.getCurrentPosition() + (int)((dist+((dist-6)/3)) * tickMM);
            target[3] = wheelBR.getCurrentPosition() + (int)((dist+((dist-6)/3)) * tickMM);
            wheelFL.setTargetPosition(target[0]);
            wheelBL.setTargetPosition(target[1]);
            wheelFR.setTargetPosition(target[2]);
            wheelBR.setTargetPosition(target[3]);

            // reset the timeout time and start motion.
            runtime.reset();

            wheelFL.setPower(Math.abs(speed));
            wheelFR.setPower(-1 * Math.abs(speed));
            wheelBL.setPower(Math.abs(speed));
            wheelBR.setPower(-1 * Math.abs(speed));

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
                telemetry.addData("Checkpoint 2", "Running to %d:, %d:, %d:, %d", target[0], target[1], target[2], target[3]);
//                telemetry.addData("Running at ", wheelFL.getCurrentPosition());
//                telemetry.addData("Running at ", wheelBL.getCurrentPosition());
//                telemetry.addData("Running at ", wheelFR.getCurrentPosition()); //100 too low ish
//                telemetry.addData("Running at ", wheelBR.getCurrentPosition()); //100 too high ish
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
