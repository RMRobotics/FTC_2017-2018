package org.firstinspires.ftc.rmrobotics.opmodes.temp8121;

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
@TeleOp(name="testServos", group="testingConfig")
public class testServos extends OpMode {

    private Servo servo1;
    private Servo servo2;

    public void init()
    {
        servo1 = hardwareMap.servo.get("servo1");
        servo2 = hardwareMap.servo.get("servo2");
//        cr1 = hardwareMap.crservo.get("cr1");
//        cr2 = hardwareMap.crservo.get("cr2");
        servo1.setPosition(0);
        servo2.setPosition(0);
//        cr1.setPower(0);
//        cr2.setPower(0);
    }

    public void loop()
    {

        if (gamepad1.right_bumper)
        {
            servo1.setPosition(1);
            servo2.setPosition(1);
//            cr1.setPower(1);
//            cr2.setPower(1);

        }
        if (gamepad1.left_bumper)
        {
            servo1.setPosition(0);
            servo2.setPosition(0);
//            cr1.setPower(-1);
//            cr2.setPower(-1);
        }
//        if (gamepad1.a){
//            cr1.setPower(0);
//            cr2.setPower(0);
//        }
    }
}
