package org.firstinspires.ftc.rmrobotics.opmodes.temp8121.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.List;
import java.util.ArrayList;
import java.util.Collections;
/**
 * Created by General on 12/5/2017.
 */
@Disabled
@TeleOp(name="motorTest", group="motorTestConfig")
public class motorTest extends OpMode {

    private DcMotor motor1;

    public void init()
    {
        motor1 = hardwareMap.dcMotor.get("motor1");
        motor1.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void loop()
    {
        motor1.setPower(gamepad1.right_stick_y);
    }
}