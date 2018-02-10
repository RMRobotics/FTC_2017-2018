package org.firstinspires.ftc.rmrobotics.opmodes.temp8121;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    private ColorSensor colorSensor;
    private ElapsedTime time = new ElapsedTime();

    public void init()
    {
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");
        wheelFL = hardwareMap.dcMotor.get("wheelFL");
        wheelFR = hardwareMap.dcMotor.get("wheelFR");
        wheelBL = hardwareMap.dcMotor.get("wheelBL");
        wheelBR = hardwareMap.dcMotor.get("wheelBR");
        wheelFL.setDirection(DcMotorSimple.Direction.REVERSE);
        wheelBL.setDirection(DcMotorSimple.Direction.REVERSE);
        colorSensor.enableLed(true);
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

        if (gamepad1.x)
        {
            boolean detected = false;
            String color = "";
            time.reset();
            while ((!detected) && (time.seconds()<5)) {
                telemetry.addData("Red  ", colorSensor.red());
                telemetry.addData("Green", colorSensor.green());
                telemetry.addData("Blue ", colorSensor.blue());
                telemetry.update();

                if ((colorSensor.blue() > 0) && (colorSensor.red()<1))
                {
                    detected = true;
                    color = "Red";
                }
                else if ((colorSensor.blue() < 1) && (colorSensor.red()>0))
                {
                    detected = true;
                    color = "Blue";
                }
                else
                    telemetry.addData("Not yet bois", "");

                telemetry.update();
            }
            telemetry.addData("Color: ",color);
            telemetry.update();
        }
    }
}
