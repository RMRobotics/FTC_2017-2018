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
@TeleOp(name="TELEBOI", group="encoderFig")
public class TELEBOI extends OpMode {

    private DcMotor wheelFL;
    private DcMotor wheelFR;
    private DcMotor wheelBL;
    private DcMotor wheelBR;

    public void init()
    {
        wheelFL = hardwareMap.dcMotor.get("wheelFL");
        wheelFR = hardwareMap.dcMotor.get("wheelFR");
        wheelBL = hardwareMap.dcMotor.get("wheelBL");
        wheelBR = hardwareMap.dcMotor.get("wheelBR");
        wheelFL.setDirection(DcMotorSimple.Direction.REVERSE);
        wheelBL.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void loop()
    {
        double forward, strafe, rotate;



        forward = -gamepad1.right_stick_y/2;
        strafe = gamepad1.right_stick_x/2;
        rotate = gamepad1.left_stick_x/2;


        double max = 1;
        /*;;;;;;;;;;;;;;;;;;;;
        List l = new ArrayList<>();
        l.add(Math.abs(forward + strafe + rotate));
        l.add(Math.abs(forward - strafe - rotate));
        l.add(Math.abs(forward - strafe + rotate));
        l.add(Math.abs(forward + strafe - rotate));*/
        wheelFL.setPower((forward + strafe + rotate) / max);
        wheelFR.setPower((forward + strafe - rotate) / max);
        wheelBL.setPower((forward - strafe + rotate) / max);
        wheelBR.setPower((forward - strafe - rotate) / max);
        /*if ((double) Collections.max(l) > 1) {
            max = (double) Collections.max(l);
        }*/
    }
}
