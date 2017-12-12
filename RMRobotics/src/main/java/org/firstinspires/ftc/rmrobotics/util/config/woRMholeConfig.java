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
    private servo clawBL;
    private servo clawBR;
    private crservo clawTL;
    private crservo clawTR;
    private servo gemBar;
    private servo relicArmT;
    private servo relicArmB;
    private motor ArmMotor;

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
        ArmMotor = new motor(hMap.dcMotor.get("ArmMotor"), DcMotorSimple.Direction.FORWARD, motorMode, DcMotor.ZeroPowerBehavior.FLOAT, MotorType.NVRST40);

        lift = new motor(hMap.dcMotor.get("lift"), DcMotorSimple.Direction.FORWARD, motorMode, DcMotor.ZeroPowerBehavior.BRAKE, MotorType.NVRST40);

        motors.addAll(Arrays.asList(wheelFL, wheelFR, wheelBL, wheelBR, lift));

        clawBL = new servo(hMap.servo.get("clawBL"), Servo.Direction.FORWARD, 0, 0.6, 0);
        clawBR = new servo(hMap.servo.get("clawBR"), Servo.Direction.REVERSE, 0, 0.6, 0);
        gemBar = new servo(hMap.servo.get("gemBar"), Servo.Direction.FORWARD, 0, 1, 0);
        relicArmT = new servo(hMap.servo.get("clawBL"), Servo.Direction.FORWARD, 0, 0.6, 0);
        relicArmB = new servo(hMap.servo.get("clawBR"), Servo.Direction.REVERSE, 0, 0.6, 0);

        servos.addAll(Arrays.asList(clawBL, clawBR, gemBar));

        clawTL = new crservo(hMap.crservo.get("clawTL"), CRServo.Direction.FORWARD);
        clawTR = new crservo(hMap.crservo.get("clawTR"), CRServo.Direction.REVERSE);

        crservos.addAll(Arrays.asList(clawTL, clawTR));

    }
    public motor wheelBR() {return wheelBR;}
    public motor wheelBL() {return wheelBL;}
    public motor wheelFL() {return wheelFL;}
    public motor wheelFR() {return wheelFR;}
    public motor lift() {return lift;}
    public servo clawBL() {return clawBL;}
    public servo clawBR() {return clawBR;}
    public crservo clawTL() {return clawTL;}
    public crservo clawTR() {return clawTR;}
    public servo gemBar() {return gemBar;}
    public servo relicArmT() {return relicArmT;}
    public servo relicArmB() {return relicArmB;}
    public motor ArmMotor() {return ArmMotor;}

}
