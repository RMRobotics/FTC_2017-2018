package org.firstinspires.ftc.rmrobotics.opmodes.temp8121.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Kameron on 1/10/2018.
 */


@Autonomous(name="testServosButBetter", group ="testingConfig")
public class testServosButBetter extends LinearOpMode {

    private Servo servo1;
    private Servo servo2;

    private ElapsedTime time = new ElapsedTime();

    @Override
    public void runOpMode() {

        servo1 = hardwareMap.servo.get("servo1");
        servo2 = hardwareMap.servo.get("servo2");

        waitForStart();
        while(opModeIsActive()) {

            servo1.setPosition(1);
            servo2.setPosition(1);

            time.reset();
            while (time.seconds() < 1){}

            servo1.setPosition(0);
            servo2.setPosition(0);

            time.reset();
            while (time.seconds() < 1){}

        }
    }
}
