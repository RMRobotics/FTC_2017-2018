package org.firstinspires.ftc.rmrobotics.opmodes.temp8121;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;

import org.firstinspires.ftc.rmrobotics.util.enums.Color;

/**
 * Created by rotom on 11/30/2017.
 */
@Autonomous(name="testingColorSensors", group ="sensorTestConfig")
public class testingColorSensor extends LinearOpMode{

    ColorSensor colorSensor;

    @Override
    public void runOpMode()
    {
        waitForStart();
        while (opModeIsActive()) {
            colorSensor = hardwareMap.colorSensor.get("cs");
            telemetry.addData("Color Sensor Red Value", colorSensor.red());
            telemetry.update();
        }
    }
}