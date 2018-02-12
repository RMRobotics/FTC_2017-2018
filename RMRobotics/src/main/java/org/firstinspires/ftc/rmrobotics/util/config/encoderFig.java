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

public class encoderFig extends Robot {

    private motor wheelBR;
    private motor wheelBL;
    private motor wheelFL;
    private motor wheelFR;

    public encoderFig(final HardwareMap h, final Op o) {
        super(h);
        opMode = o;
    }

    public encoderFig(final HardwareMap h, final DcMotor.RunMode runMode) {
        super(h);
        motorMode = runMode;
    }

    @Override
    protected void config() {

        wheelFL = new motor(hMap.dcMotor.get("wheelFL"), DcMotorSimple.Direction.REVERSE, motorMode, DcMotor.ZeroPowerBehavior.FLOAT, MotorType.NVRST40);
        wheelFR = new motor(hMap.dcMotor.get("wheelFR"), DcMotorSimple.Direction.FORWARD, motorMode, DcMotor.ZeroPowerBehavior.FLOAT, MotorType.NVRST40);
        wheelBL = new motor(hMap.dcMotor.get("wheelBL"), DcMotorSimple.Direction.REVERSE, motorMode, DcMotor.ZeroPowerBehavior.FLOAT, MotorType.NVRST40);
        wheelBR = new motor(hMap.dcMotor.get("wheelBR"), DcMotorSimple.Direction.FORWARD, motorMode, DcMotor.ZeroPowerBehavior.FLOAT, MotorType.NVRST40);
        motors.addAll(Arrays.asList(wheelFL, wheelFR, wheelBL, wheelBR));
    }

    public motor wheelBR() {return wheelBR;}
    public motor wheelBL() {return wheelBL;}
    public motor wheelFL() {return wheelFL;}
    public motor wheelFR() {return wheelFR;}

}
