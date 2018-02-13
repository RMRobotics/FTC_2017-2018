package org.firstinspires.ftc.rmrobotics.opmodes.temp8121.tests;

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

@Autonomous(name="test123", group ="woRMholeConfig")
public class test123 extends LinearOpMode {

    //initialization of navx stuff
    private final int NAVX_DIM_I2C_PORT = 0; //port on the dim
    private AHRS navx_device;
    private navXPIDController yawPIDController;

    private DcMotor wheelFL;
    private DcMotor wheelFR;
    private DcMotor wheelBL;
    private DcMotor wheelBR;

    private ElapsedTime runtime = new ElapsedTime();


    @Override public void runOpMode() throws InterruptedException {

        //hardware map in wormhole config in the UTIL folder
        wheelFL = hardwareMap.dcMotor.get("wheelFL");
        wheelFR = hardwareMap.dcMotor.get("wheelFR");
        wheelBL = hardwareMap.dcMotor.get("wheelBL");
        wheelBR = hardwareMap.dcMotor.get("wheelBR");

        float mmPerInch        = 25.4f;
        float mmBotWidth       = 18 * mmPerInch;            // ... or whatever is right for your robot
        float mmFTCFieldWidth  = (12*12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels



        waitForStart();
        while (opModeIsActive()) {

            rotateToAngle(90.0);
            stop();

        }
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
                    telemetry.addData("Config on point yeet", "");
                } else {
                    double output = yawPIDResult.getOutput();
                    if ( output < 0 ) {
                /* Rotate Left */
                        wheelFL.setPower(-output);
                        wheelFR.setPower(output);
                        wheelBL.setPower(-output);
                        wheelBR.setPower(output);
                        telemetry.addData("configuring L:", output);
                    } else {
                /* Rotate Right */
                        wheelFL.setPower(output);
                        wheelFR.setPower(-output);
                        wheelBL.setPower(output);
                        wheelBR.setPower(-output);
                        telemetry.addData("configuring R:", output);
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
        double strafe = 0;
        double max = 1;

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
                        wheelFL.setPower((drive_speed + strafe + output) / max);
                        wheelFR.setPower((drive_speed + strafe - output) / max);
                        wheelBL.setPower((drive_speed - strafe + output) / max);
                        wheelBR.setPower((drive_speed - strafe - output) / max);
                        telemetry.addData("turning L", output);
                    } else {
                        /* Rotate Right */
                        wheelFL.setPower((drive_speed + strafe - output) / max);
                        wheelFR.setPower((drive_speed + strafe + output) / max);
                        wheelBL.setPower((drive_speed - strafe - output) / max);
                        wheelBR.setPower((drive_speed - strafe + output) / max);
                        telemetry.addData("turning R", output);
                    }
                }
            } else {
			    /* A timeout occurred */
                Log.w("navXRotateToAnglePIDOp", "Yaw PID waitForNewUpdate() TIMEOUT.");
            }
        }
    }
}


