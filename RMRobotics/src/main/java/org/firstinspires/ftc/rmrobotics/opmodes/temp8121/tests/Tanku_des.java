package org.firstinspires.ftc.rmrobotics.opmodes.temp8121.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by rotom on 9/26/2017.
 */

@Disabled
@TeleOp(name="tanku8121", group="temp8121")
public class Tanku_des extends OpMode {
//    private DcMotor wheelFL;
//    private DcMotor wheelFR;
    private DcMotor wheelBL;
    private DcMotor wheelBR;
    private DcMotor arm;
    private Servo clawL;
    private Servo clawR;
    private boolean clawState = false;

    @Override
    public void init() {
//weebaoo
//        wheelFL = hardwareMap.dcMotor.get("wheelFL");
//        wheelFR = hardwareMap.dcMotor.get("wheelFR");
        wheelBL = hardwareMap.dcMotor.get("wheelBL");
        wheelBR = hardwareMap.dcMotor.get("wheelBR");
//        wheelFL.setDirection(DcMotorSimple.Direction.REVERSE);
        wheelBL.setDirection(DcMotorSimple.Direction.REVERSE);
        arm = hardwareMap.dcMotor.get("arm");
        clawL = hardwareMap.servo.get("clawL");
        clawR = hardwareMap.servo.get("clawR");
        clawL.setPosition(0);
        clawR.setPosition(0);

    }

    @Override
    public void loop() {

        double forward = gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        if (turn == 0){
//            wheelFL.setPower(gamepad1.left_stick_y);
//            wheelFR.setPower(gamepad1.left_stick_y);
            wheelBL.setPower(gamepad1.left_stick_y);
            wheelBR.setPower(gamepad1.left_stick_y);
        }
        else if(forward == 0)
        {
//            wheelFL.setPower(-1*gamepad1.right_stick_x);
//            wheelFR.setPower(gamepad1.right_stick_x);
            wheelBL.setPower(-1*gamepad1.right_stick_x);
            wheelBR.setPower(gamepad1.right_stick_x);
        }
        if (gamepad1.right_bumper)
            arm.setPower(50);
        if (gamepad1.left_bumper)
            arm.setPower(-50);
        if (gamepad1.a && !clawState) {
            clawL.setPosition(1);
            clawR.setPosition(1);
            clawState = !clawState;
        }
        if (gamepad1.a && clawState) {
            clawL.setPosition(0);
            clawR.setPosition(0);
            clawState = !clawState;
        }
    }
}
