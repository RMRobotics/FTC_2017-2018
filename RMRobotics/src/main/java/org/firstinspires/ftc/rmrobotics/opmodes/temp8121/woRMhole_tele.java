package org.firstinspires.ftc.rmrobotics.opmodes.temp8121;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.rmrobotics.hardware.motor;
import org.firstinspires.ftc.rmrobotics.hardware.servo;

import java.util.List;
import java.util.ArrayList;
import java.util.Collections;
/**
 * Created by General on 12/5/2017.
 */
@TeleOp(name="woRMhole_tele", group="temp8121")
public class woRMhole_tele extends OpMode {

    private DcMotor wheelFL;
    private DcMotor wheelFR;
    private DcMotor wheelBL;
    private DcMotor wheelBR;
    private DcMotor lift;
    private Servo clawBL;
    private Servo clawBR;
    private CRServo clawTL;
    private CRServo clawTR;
    private Servo gemBar;
    private Servo relicArmT;
    private Servo relicArmB;
    private DcMotor ArmMotor;
    private boolean clawState = false;

    public void init()
    {
        wheelFL = hardwareMap.dcMotor.get("wheelFL");
        wheelFR = hardwareMap.dcMotor.get("wheelFR");
        wheelBL = hardwareMap.dcMotor.get("wheelBL");
        wheelBR = hardwareMap.dcMotor.get("wheelBR");
        ArmMotor = hardwareMap.dcMotor.get("ArmMotor");
        wheelFL.setDirection(DcMotorSimple.Direction.REVERSE);
        wheelBL.setDirection(DcMotorSimple.Direction.REVERSE);
        lift = hardwareMap.dcMotor.get("lift");
        clawBL = hardwareMap.servo.get("clawBL");
        clawBR = hardwareMap.servo.get("clawBR");
        clawTL = hardwareMap.crservo.get("clawTL");
        clawTR = hardwareMap.crservo.get("clawTR");
        relicArmT = hardwareMap.servo.get("relicArmT");
        relicArmB = hardwareMap.servo.get("relicArmB");
        clawBR.setDirection(Servo.Direction.REVERSE);
        clawTR.setDirection(CRServo.Direction.REVERSE);
        relicArmB.setDirection(Servo.Direction.REVERSE);
        gemBar = hardwareMap.servo.get("gemBar");
        clawBL.setPosition(0);
        clawBR.setPosition(0);
        clawTL.setPower(0);
        clawTR.setPower(0);
        gemBar.setPosition(0);
    }

    public void loop()
    {
        double forward = gamepad1.right_stick_y;
        double strafe = gamepad1.right_stick_x;
        double rotate = gamepad1.left_stick_x;
        double max = 1;
        List l = new ArrayList<>();
        l.add(Math.abs(forward + strafe + rotate));
        l.add(Math.abs(forward - strafe - rotate));
        l.add(Math.abs(forward - strafe + rotate));
        l.add(Math.abs(forward + strafe - rotate));
        if ((double) Collections.max(l) > 1) {
            max = (double) Collections.max(l);
        }
        wheelFL.setPower((-forward + strafe + rotate) / max);
        wheelFR.setPower((-forward - strafe - rotate) / max);
        wheelBL.setPower((-forward - strafe + rotate) / max);
        wheelBR.setPower((-forward + strafe - rotate) / max);

        if (gamepad2.right_bumper)
        {
            clawBL.setPosition(0.6);
            clawBR.setPosition(0.6);

        }
        if (gamepad2.left_bumper)
        {
            clawBL.setPosition(0);
            clawBR.setPosition(0);
        }
        if (gamepad2.left_trigger!=0 && gamepad2.right_trigger==0)
        {
            clawTR.setPower(-gamepad2.left_trigger/128);
            clawTL.setPower(-gamepad2.left_trigger/128);


        }
        else
        {
            clawTR.setPower(0);
            clawTL.setPower(0);
        }
        if (gamepad2.right_trigger!=0 && gamepad2.left_trigger==0)
        {
            clawTR.setPower(gamepad2.left_trigger/128);
            clawTL.setPower(gamepad2.left_trigger/128);
        }

        ArmMotor.setPower(gamepad2.left_stick_y/128);
        lift.setPower(gamepad2.right_stick_y/128);

        if (gamepad2.a)
        {
            gemBar.setPosition(1);

        }
        if (gamepad2.b)
        {
            gemBar.setPosition(-1);
        }
        if (gamepad2.x)
        {
            relicArmB.setPosition(0.6);
            relicArmT.setPosition(0.6);
        }
        if (gamepad2.y)
        {
            relicArmB.setPosition(0);
            relicArmT.setPosition(0);
        }
    }
}
