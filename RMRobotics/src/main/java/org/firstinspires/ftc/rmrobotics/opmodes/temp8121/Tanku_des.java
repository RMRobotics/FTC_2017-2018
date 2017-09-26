package org.firstinspires.ftc.rmrobotics.opmodes.temp8121;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by rotom on 9/26/2017.
 */

@TeleOp(name="tanku8121", group="temp8121")
public class Tanku_des extends OpMode {
    private DcMotor wheelFL;
    private DcMotor wheelFR;
    private DcMotor wheelBL;
    private DcMotor wheelBR;
    private DcMotor arm;
    private Servo grabberL;
    private Servo grabberR;
    boolean claw = false;

    @Override
    public void init() {

        wheelFL = hardwareMap.dcMotor.get("wheelFL");
        wheelFR = hardwareMap.dcMotor.get("wheelFR");
        wheelBL = hardwareMap.dcMotor.get("wheelBL");
        wheelBR = hardwareMap.dcMotor.get("wheelBR");
        wheelFL.setDirection(DcMotorSimple.Direction.REVERSE);
        wheelBL.setDirection(DcMotorSimple.Direction.REVERSE);
        wheelBR = hardwareMap.dcMotor.get("arm");
        grabberL = hardwareMap.servo.get("grabberL");
        grabberR = hardwareMap.servo.get("grabberR");
        grabberL.setPosition(0);
        grabberR.setPosition(0);

    }

    @Override
    public void loop() {

        double forward = gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        if (turn == 0){
            wheelFL.setPower(gamepad1.left_stick_y);
            wheelFR.setPower(gamepad1.left_stick_y);
            wheelBL.setPower(gamepad1.left_stick_y);
            wheelBR.setPower(gamepad1.left_stick_y);
        }
        else if(forward == 0)
        {
            wheelFL.setPower(-1*gamepad1.right_stick_x);
            wheelFR.setPower(gamepad1.right_stick_x);
            wheelBL.setPower(-1*gamepad1.right_stick_x);
            wheelBR.setPower(gamepad1.right_stick_x);
        }
        if (gamepad1.right_bumper)
            arm.setPower(50);
        if (gamepad1.left_bumper)
            arm.setPower(-50);
        if (gamepad1.a && !claw) {
            grabberL.setPosition(1);
            grabberR.setPosition(1);
        }
        if (gamepad1.a && claw) {
            grabberL.setPosition(0);
            grabberR.setPosition(0);
        }
    }
}
