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

public class woRMholeConfig extends Robot {

    private motor wheelBR;
    private motor wheelBL;
    private motor wheelFL;
    private motor wheelFR;
    private motor lift;
    private motor arm;
    private servo clawBL;
    private servo clawBR;
    private servo clawTL;
    private servo clawTR;
    private servo armT;
    private crservo armB;
//    private servo gemBar;

    public woRMholeConfig(final HardwareMap h, final Op o) {
        super(h);
        opMode = o;
    }

    public woRMholeConfig(final HardwareMap h, final DcMotor.RunMode runMode) {
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
        wheelFL = new motor(hMap.dcMotor.get("wheelFL"), DcMotorSimple.Direction.REVERSE, motorMode, DcMotor.ZeroPowerBehavior.FLOAT, MotorType.NVRST40);
        wheelFR = new motor(hMap.dcMotor.get("wheelFR"), DcMotorSimple.Direction.FORWARD, motorMode, DcMotor.ZeroPowerBehavior.FLOAT, MotorType.NVRST40);
        wheelBL = new motor(hMap.dcMotor.get("wheelBL"), DcMotorSimple.Direction.REVERSE, motorMode, DcMotor.ZeroPowerBehavior.FLOAT, MotorType.NVRST40);
        wheelBR = new motor(hMap.dcMotor.get("wheelBR"), DcMotorSimple.Direction.FORWARD, motorMode, DcMotor.ZeroPowerBehavior.FLOAT, MotorType.NVRST40);
        lift = new motor(hMap.dcMotor.get("lift"), DcMotorSimple.Direction.REVERSE, motorMode, DcMotor.ZeroPowerBehavior.BRAKE, MotorType.NVRST20);
        arm = new motor(hMap.dcMotor.get("arm"), DcMotorSimple.Direction.FORWARD, motorMode, DcMotor.ZeroPowerBehavior.BRAKE, MotorType.NVRST20);
        motors.addAll(Arrays.asList(wheelFL, wheelFR, wheelBL, wheelBR, lift, arm));

        clawBL = new servo(hMap.servo.get("clawBL"), Servo.Direction.FORWARD, -1, 1, -1);
        clawBR = new servo(hMap.servo.get("clawBR"), Servo.Direction.FORWARD, -1, 1, -1);
        armT = new servo(hMap.servo.get("armT"), Servo.Direction.FORWARD, -1, 1, 0);
        clawTL = new servo(hMap.servo.get("clawTL"), Servo.Direction.FORWARD, -1, 1, 0);
        clawTR = new servo(hMap.servo.get("clawTR"), Servo.Direction.FORWARD, -1, 1, 0);

//        gemBar = new servo(hMap.servo.get("gemBar"), Servo.Direction.FORWARD, 0, 1, 0);

        servos.addAll(Arrays.asList(clawBL, clawBR, /*gemBar, */armT, clawTL, clawTR));

        armB = new crservo(hMap.crservo.get("armB"), CRServo.Direction.FORWARD);

        crservos.addAll(Arrays.asList(armB));

    }
    public motor wheelBR() {return wheelBR;}
    public motor wheelBL() {return wheelBL;}
    public motor wheelFL() {return wheelFL;}
    public motor wheelFR() {return wheelFR;}
    public motor lift() {return lift;}
    public motor arm() {return arm;}
    public servo clawBL() {return clawBL;}
    public servo clawBR() {return clawBR;}
    public servo armT() {return armT;}
    public crservo armB() {return armB;}
    public servo clawTL() {return clawTL;}
    public servo clawTR() {return clawTR;}
//    public servo gemBar() {return gemBar;}

}
