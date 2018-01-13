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
//    private DcMotor arm;
    private Servo clawBL;
    private Servo clawBR;
    private Servo clawTL;
    private Servo clawTR;
    private Servo armT;
    private CRServo armB;
//    private Servo gemBar;
    private boolean clawState = false, slowMo = false;

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
        lift.setPower(0);
//        arm = hardwareMap.dcMotor.get("arm");
        clawBL = hardwareMap.servo.get("clawBL");
        clawBR = hardwareMap.servo.get("clawBR");
        armT = hardwareMap.servo.get("armT");
        armB = hardwareMap.crservo.get("armB");
        clawTL = hardwareMap.servo.get("clawTL");
        clawTR = hardwareMap.servo.get("clawTR");
        armB.setDirection(CRServo.Direction.FORWARD);
//        gemBar = hardwareMap.servo.get("gemBar");
        clawBL.setPosition(-0.7);
        clawBR.setPosition(1);
        armT.setPosition(0.5);
        armB.setPower(0);
        clawTL.setPosition(-1);
        clawTR.setPosition(1);
//        gemBar.setPosition(0);
    }

    public void loop()
    {
        double forward, strafe, rotate;

        if (slowMo)
        {
            forward = -gamepad1.right_stick_y/3;
            strafe = -gamepad1.right_stick_x/3;
            rotate = gamepad1.left_stick_x/3;
        }
        else {
            forward = -gamepad1.right_stick_y;
            strafe = -gamepad1.right_stick_x;
            rotate = gamepad1.left_stick_x;
        }

        double max = 1;
        /*;;;;;;;;;;;;;;;;;;;;
        List l = new ArrayList<>();
        l.add(Math.abs(forward + strafe + rotate));
        l.add(Math.abs(forward - strafe - rotate));
        l.add(Math.abs(forward - strafe + rotate));
        l.add(Math.abs(forward + strafe - rotate));*/
        wheelFL.setPower((forward + strafe + rotate) / max);
        wheelFR.setPower((forward - strafe - rotate) / max);
        wheelBL.setPower((forward - strafe + rotate) / max);
        wheelBR.setPower((forward + strafe - rotate) / max);
        /*if ((double) Collections.max(l) > 1) {
            max = (double) Collections.max(l);
        }*/
        if (gamepad1.right_bumper)
        {
            clawTR.setPosition(0);
            clawTL.setPosition(0.75);
        }
        if (gamepad1.left_bumper)
        {
            clawTR.setPosition(1);
            clawTL.setPosition(-1);
        }
/*        if (gamepad2.x){
            clawTL.setPower(1);
            clawTR.setPower(1);
        }
        if (gamepad2.y){
            clawTR.setPower(-1);
            clawTL.setPower(-1);

        }*/
        if (gamepad1.right_trigger != 0)
        {
            clawBL.setPosition(0.7);
            clawBR.setPosition(0.3);
        }
        if (gamepad1.left_trigger != 0)
        {

            clawBL.setPosition(-0.7);
            clawBR.setPosition(1);
        }

        if (gamepad2.right_stick_y < 0)
            lift.setPower(gamepad2.right_stick_y/3);
        else
            lift.setPower(gamepad2.right_stick_y/2);

//        arm.setPower(gamepad2.left_stick_y/2);
        if (gamepad2.y)
        {
            armT.setPosition(0);
        }
        if (gamepad2.x)
        {
            armT.setPosition(0.75);
        }
        if (gamepad2.left_trigger > 0 && gamepad2.right_trigger == 0)
        {
            armB.setPower(-1);
        }
        else if (gamepad2.right_trigger > 0 && gamepad2.left_trigger == 0)
        {
            armB.setPower(1);
        }
        else{
            armB.setPower(0);
        }
        if (gamepad1.b){
            slowMo = !slowMo;
        }

//        if (gamepad1.a)
//        {
//            gemBar.setPosition(0);
//        }
//        if (gamepad1.b)
//        {
//            gemBar.setPosition(1);
//        }
    }
}
