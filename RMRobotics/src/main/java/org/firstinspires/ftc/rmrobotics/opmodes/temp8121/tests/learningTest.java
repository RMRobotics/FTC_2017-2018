package org.firstinspires.ftc.rmrobotics.opmodes.temp8121.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
/**
 * Created by rotom on 9/28/2017.
 */
@Disabled
@TeleOp (name="test", group="8121")
public class learningTest extends OpMode {

    private DcMotor wheel1;
    private DcMotor wheel2;
    private DcMotor wheel3;
    private DcMotor wheel4;


    public void init(){
        wheel1 = hardwareMap.dcMotor.get("wheel1");
        wheel2 = hardwareMap.dcMotor.get("wheel2");
        wheel3 = hardwareMap.dcMotor.get("wheel3");
        wheel4 = hardwareMap.dcMotor.get("wheel4");
        wheel1.setDirection(DcMotorSimple.Direction.REVERSE);
        wheel3.setDirection(DcMotorSimple.Direction.REVERSE);


    }

    public void loop() {
        double forward = gamepad1.left_stick_y;

        if (forward > 0){
            wheel1.setPower(gamepad1.left_stick_y/128);
            wheel2.setPower(gamepad1.left_stick_y/128);
            wheel3.setPower(gamepad1.left_stick_y/128);
            wheel4.setPower(gamepad1.left_stick_y/128);
        }

        if (forward < 0){
            wheel1.setPower(-1*gamepad1.left_stick_y/128);
            wheel2.setPower(-1*gamepad1.left_stick_y/128);
            wheel3.setPower(-1*gamepad1.left_stick_y/128);
            wheel4.setPower(-1*gamepad1.left_stick_y/128);
        }


    }




}
