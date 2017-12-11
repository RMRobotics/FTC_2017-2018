package org.firstinspires.ftc.rmrobotics.util.config;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
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
 * Created by rotom on 9/26/2017.
 */

public class temp8121 extends Robot {

//    private motor wheelFL;
//    private motor wheelFR;
    private motor wheelBL;
    private motor wheelBR;
    private motor arm;
    private servo clawL;
    private servo clawR;

    public temp8121(final HardwareMap h, final Op o) {
        super(h);
        opMode = o;
    }

    public temp8121(final HardwareMap h, final DcMotor.RunMode runMode) {
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
//        wheelFL = new motor(hMap.dcMotor.get("wheelFL"), DcMotorSimple.Direction.REVERSE, motorMode, DcMotor.ZeroPowerBehavior.FLOAT, MotorType.NVRST40);
//        wheelFR = new motor(hMap.dcMotor.get("wheelFR"), DcMotorSimple.Direction.FORWARD, motorMode, DcMotor.ZeroPowerBehavior.FLOAT, MotorType.NVRST40);
        wheelBL = new motor(hMap.dcMotor.get("wheelBL"), DcMotorSimple.Direction.REVERSE, motorMode, DcMotor.ZeroPowerBehavior.FLOAT, MotorType.NVRST40);
        wheelBR = new motor(hMap.dcMotor.get("wheelBR"), DcMotorSimple.Direction.FORWARD, motorMode, DcMotor.ZeroPowerBehavior.FLOAT, MotorType.NVRST40);
        arm = new motor(hMap.dcMotor.get("arm"), DcMotorSimple.Direction.FORWARD, motorMode, DcMotor.ZeroPowerBehavior.BRAKE, MotorType.NVRST40);
        motors.addAll(Arrays.asList(/*wheelFL, wheelFR, */wheelBL, wheelBR, arm));

        clawL = new servo(hMap.servo.get("clawL"), Servo.Direction.FORWARD, 0, 0.3, 0);
        clawR = new servo(hMap.servo.get("clawR"), Servo.Direction.REVERSE, 0, 0.3, 0);
        servos.addAll(Arrays.asList(clawL, clawR));
    }

//    public motor wheelFL() { return wheelFL; }
//    public motor wheelFR() { return wheelFR; }
    public motor wheelBL() { return wheelBL; }
    public motor wheelBR() { return wheelBR; }
    public motor arm() { return arm; }
    public servo clawL() { return clawL; }
    public servo clawR() { return clawR; }
}
