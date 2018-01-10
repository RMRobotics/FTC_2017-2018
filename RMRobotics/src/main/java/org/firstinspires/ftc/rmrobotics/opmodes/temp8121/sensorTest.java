//package org.firstinspires.ftc.rmrobotics.opmodes.temp8121;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.ColorSensor;
//
//import org.firstinspires.ftc.rmrobotics.util.enums.Color;
//
///**
// * Created by rotom on 10/17/2017.
// */
//
//@Autonomous(name="sensorTest", group ="sensorTestConfig")
//public class sensorTest extends LinearOpMode{
//
//    I2cDevice sensor;
//    I2cDeviceSynch sensor;
//    @Override public void runOpMode() {
//
//        Color colorboi;
//        while(true) {
//            if (sensor.read(0x04, 1)[0] >= 6)
//                colorboi = Color.RED;
//            else if (sensor.read(0x04, 1)[0] < 6)
//                colorboi = Color.BLUE;
//            else
//                colorboi = Color.NEITHER;
//            telemetry.addData("Color is:", colorboi);
//        }
//    }
//}
