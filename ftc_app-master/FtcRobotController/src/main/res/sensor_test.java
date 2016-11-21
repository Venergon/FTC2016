package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.LightSensor;

/**
 * Created by Segway on 29-Oct-16.
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="EmuOp: Auto", group="EmuOp")

public class Auto extends OpMode {
    AnalogInput sensor;

    public void init(){
        sensor = hardwareMap.lightSensor.get("sensors");
    }

    public void init_start() {

    }

    public void loop() {
        System.out.print(sensor.getVoltage());
    }


}