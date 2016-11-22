package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorManager;
import android.os.Looper;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Segway on 21-Nov-16.
 */

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="EmuOp: Test", group="EmuOp")

public class Test extends OpMode {
    /*boolean first;
    AnalogInput sensorTest;
    SensorManager sensorManager;
    MyGyro gyro;*/
    AnalogInput lightSensor;

    public void init() {
        /*Looper.prepare();
        sensorManager = (SensorManager)hardwareMap.appContext.getSystemService(Context.SENSOR_SERVICE);
        gyro =  new MyGyro();
        sensorManager.registerListener(gyro, sensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR), SensorManager.SENSOR_DELAY_NORMAL);
        first = true;
        //sensorTest = hardwareMap.analogInput.get("sensors");*/
        lightSensor = hardwareMap.analogInput.get("sensors");
    }

    public void init_loop() {

    }

    public void start() {

    }

    @Override
    public void loop() {
        /*if (first) {
            Looper.loop();
            first = false;
        }*/
        //double val = gyro.getAngle();
        //telemetry.addData("sensor", Double.toString(val));
        telemetry.addData("Light sensor", lightSensor.getVoltage());
    }
}
