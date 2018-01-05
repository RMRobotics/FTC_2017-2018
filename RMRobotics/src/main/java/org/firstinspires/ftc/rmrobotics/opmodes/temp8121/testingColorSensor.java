package org.firstinspires.ftc.rmrobotics.opmodes.temp8121;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;

import org.firstinspires.ftc.rmrobotics.util.enums.Color;

/**
 * Created by rotom on 11/30/2017.
 */
@Autonomous(name="testColorSensors", group ="dummyConfig")
public class testingColorSensor extends LinearOpMode{

    I2cDevice colorSensor;
    I2cDeviceSynch colorSensorReader;

    @Override
    public void runOpMode()
    {

        telemetry.addData(">", "Press Play to start");
        telemetry.update();

        String color = "";
        Color cc;
        byte[] colorCache;
        try{
            colorSensor = hardwareMap.i2cDevice.get("colorSensor");
            colorSensorReader = new I2cDeviceSynchImpl(colorSensor, I2cAddr.create8bit(0x52), false);
            colorSensorReader.engage();
            colorSensorReader.write8(3,0);

            while(true)
            {
                String cololor = "";
                Color colorboi;
                if (colorSensorReader.read(0x04, 1)[0] >=6)
                {    colorboi = Color.RED;
                    cololor = "red";}

                else
                {
                    colorboi = Color.BLUE;
                    cololor = "blue";}
                telemetry.addData("Color is", cololor);
            }

//            colorCache =colorSensorReader.read(0x05, 1);
            //if detects red
//                if (colorCache[0] >= 3)
//                    color = "Red";
//                    //if detects blue
//                else
//                    color = "Blue";
//            if (colorCache[0] > 0) {
//                cc = Color.RED;
//            } else {
//                cc = Color.BLUE;
//            }
//            telemetry.addData("Color is", colorCache[0]);
//            sleep(3000);
        }

        catch (NullPointerException e){
            telemetry.addData("yur mum gey!?", "");
        }
        telemetry.update();
    }
}