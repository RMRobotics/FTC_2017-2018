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
@TeleOp(name="woRMhole_tele", group="woRMholeConfig")
public class woRMhole_tele extends OpMode {

    private DcMotor wheelFL;
    private DcMotor wheelFR;
    private DcMotor wheelBL;
    private DcMotor wheelBR;
    private DcMotor lift;
    private DcMotor arm;
    private Servo clawBL;
    private Servo clawBR;
    private CRServo clawTL;
    private CRServo clawTR;
    private Servo armT;
    private Servo armB;
    private Servo gemBar;
    private boolean clawState = false;

    public void init()
    {
        wheelFL = hardwareMap.dcMotor.get("wheelFL");
        wheelFR = hardwareMap.dcMotor.get("wheelFR");
        wheelBL = hardwareMap.dcMotor.get("wheelBL");
        wheelBR = hardwareMap.dcMotor.get("wheelBR");
        wheelFL.setDirection(DcMotorSimple.Direction.REVERSE);
        wheelBL.setDirection(DcMotorSimple.Direction.REVERSE);
        lift = hardwareMap.dcMotor.get("lift");
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        arm = hardwareMap.dcMotor.get("arm");
        clawBL = hardwareMap.servo.get("clawBL");
        clawBR = hardwareMap.servo.get("clawBR");
        armT = hardwareMap.servo.get("armT");
        armB = hardwareMap.servo.get("armB");
        clawTL = hardwareMap.crservo.get("clawTL");
        clawTR = hardwareMap.crservo.get("clawTR");
        clawBR.setDirection(Servo.Direction.REVERSE);
        clawTR.setDirection(CRServo.Direction.REVERSE);
        armB.setDirection(Servo.Direction.REVERSE);
        gemBar = hardwareMap.servo.get("gemBar");
        clawBL.setPosition(0);
        clawBR.setPosition(0);
        armT.setPosition(0);
        armB.setPosition(0);
        clawTL.setPower(0);
        clawTR.setPower(0);
        gemBar.setPosition(0);
    }

    public void loop()
    {
        double forward, strafe, rotate;

        if (gamepad1.right_trigger!=0)
        {
            forward = gamepad1.right_stick_y/3;
            strafe = gamepad1.right_stick_x/3;
            rotate = gamepad1.left_stick_x/3;
        }
        else {
            forward = gamepad1.right_stick_y;
            strafe = gamepad1.right_stick_x;
            rotate = gamepad1.left_stick_x;
        }

        double max = 1;
        List l = new ArrayList<>();
        l.add(Math.abs(forward + strafe + rotate));
        l.add(Math.abs(forward - strafe - rotate));
        l.add(Math.abs(forward - strafe + rotate));
        l.add(Math.abs(forward + strafe - rotate));
        if ((double) Collections.max(l) > 1) {
            max = (double) Collections.max(l);
        }
        if (gamepad2.right_bumper)
        {
            clawBL.setPosition(0);
            clawBR.setPosition(0);
        }
        if (gamepad2.left_bumper)
        {
            clawBL.setPosition(0.6);
            clawBR.setPosition(0.6);
        }
        if (gamepad2.right_trigger!=0 && gamepad2.left_trigger==0)
        {
            clawTR.setPower(gamepad2.left_trigger/128);
            clawTL.setPower(gamepad2.left_trigger/128);
        }
        else if (gamepad2.left_trigger!=0 && gamepad2.right_trigger==0)
        {
            clawTR.setPower(-gamepad2.left_trigger/128);
            clawTL.setPower(-gamepad2.left_trigger/128);
        }
        else
        {
            clawTR.setPower(0);
            clawTL.setPower(0);
        }
        lift.setPower(gamepad2.right_stick_y/2);
        arm.setPower(gamepad2.left_stick_y/2);
        if (gamepad2.a)
        {
            armT.setPosition(0);
            armB.setPosition(0);
        }
        if (gamepad2.b)
        {
            armT.setPosition(0.6);
            armB.setPosition(0.6);
        }
        if (gamepad1.a)
        {
            gemBar.setPosition(0);
        }
        if (gamepad1.b)
        {
            gemBar.setPosition(1);
        }
    }
}
