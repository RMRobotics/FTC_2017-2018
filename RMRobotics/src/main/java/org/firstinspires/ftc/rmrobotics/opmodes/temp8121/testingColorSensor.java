package org.firstinspires.ftc.rmrobotics.opmodes.temp8121;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

import org.firstinspires.ftc.rmrobotics.util.enums.Color;

/**
 * Created by rotom on 11/30/2017.
 */
@Autonomous(name="testColorSensor", group ="temp8121")
public class testingColorSensor extends LinearOpMode{
    I2cDeviceSynch sensor;
    public void runOpMode()
    {
        telemetry.addData(">", "Press Play to start");
        telemetry.update();
//        while(opModeIsActive())
//        {
            Color colorboi;
            if (sensor.read(0x04, 1)[0] >=6)
                colorboi = Color.RED;
            else
                colorboi = Color.BLUE;
            telemetry.addData("Color is", colorboi);
//        }
    }
}
