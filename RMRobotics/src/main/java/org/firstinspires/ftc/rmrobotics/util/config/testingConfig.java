package org.firstinspires.ftc.rmrobotics.util.config;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.rmrobotics.hardware.i2csensor;
import org.firstinspires.ftc.rmrobotics.hardware.crservo;
import org.firstinspires.ftc.rmrobotics.hardware.motor;
import org.firstinspires.ftc.rmrobotics.hardware.servo;
import org.firstinspires.ftc.rmrobotics.util.enums.MotorType;
import org.firstinspires.ftc.rmrobotics.util.enums.Op;
import org.firstinspires.ftc.rmrobotics.util.enums.Sensors;

import java.util.Arrays;

/**
 * Created by General on 12/5/2017.
 */

public class testingConfig extends Robot {

    private servo servo1;
    private servo servo2;
//    private crservo cr1;
//    private crservo cr2;

    public testingConfig(final HardwareMap h, final Op o) {
        super(h);
        opMode = o;
    }

    public testingConfig(final HardwareMap h, final DcMotor.RunMode runMode) {
        super(h);
        motorMode = runMode;
    }

    @Override
    protected void config() {
        if (opMode != null) {
            if (opMode == Op.TELEOP) {
                motorMode = DcMotor.RunMode.RUN_USING_ENCODER;
            } else {
                motorMode = DcMotor.RunMode.RUN_TO_POSITION;
            }
        }


        servo1 = new servo(hMap.servo.get("servo1"), Servo.Direction.FORWARD, 0, 0.6, 0);
        servo2 = new servo(hMap.servo.get("servo2"), Servo.Direction.FORWARD, 0, 0.6, 0);

        servos.addAll(Arrays.asList(servo1,servo2));

//        cr1 = new crservo(hMap.crservo.get("cr1"), CRServo.Direction.FORWARD);
//        cr2 = new crservo(hMap.crservo.get("cr2"), CRServo.Direction.FORWARD);
//
//        crservos.addAll(Arrays.asList(cr1, cr2));

    }

    public servo servo1() {return servo1;}
    public servo servo2() {return servo2;}
//    public crservo cr1() {return cr1;}
//    public crservo cr2() {return cr2;}

}
