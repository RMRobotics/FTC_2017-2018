package org.firstinspires.ftc.rmrobotics.util.config;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.rmrobotics.util.enums.Op;
import org.firstinspires.ftc.rmrobotics.hardware.i2csensor;
import org.firstinspires.ftc.rmrobotics.util.enums.Sensors;
import java.util.Arrays;

/**
 * Created by rotom on 12/15/2017.
 */

public class sensorTestConfig extends Robot {

    private i2csensor colorSensor;

    public sensorTestConfig(final HardwareMap h, final Op o) {
        super(h);
        opMode = o;
    }
    protected void config() {
        colorSensor = new i2csensor(hMap.i2cDevice.get("colorSensor"), (byte) 0x52, Sensors.COLORON);
        sensors.addAll(Arrays.asList(colorSensor));
    }
    public i2csensor colorSesnor() {return colorSensor;}
}