package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorManager;
import android.os.Looper;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
 * Created by Segway on 21-Nov-16.
*/


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="EmuOp: Test", group="EmuOp")

public class Test extends OpMode {
    //boolean first;
    //AnalogInput sensorTest;
    //SensorManager sensorManager;
    //MyGyro gyro;
    AnalogInput lightSensor;
    //Servo servoTest;
    DcMotor leftBackDrive;
    DcMotor leftForwardDrive;
    DcMotor rightBackDrive;
    DcMotor rightForwardDrive;

    public void init() {
        //Looper.prepare();
        //sensorManager = (SensorManager)hardwareMap.appContext.getSystemService(Context.SENSOR_SERVICE);
        //gyro =  new MyGyro();
        //sensorManager.registerListener(gyro, sensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR), SensorManager.SENSOR_DELAY_NORMAL);
        //first = true;

        //sensorTest = hardwareMap.analogInput.get("sensors");
        lightSensor = hardwareMap.analogInput.get("light_sensor");
        leftBackDrive = hardwareMap.dcMotor.get("left_back_drive");
        leftForwardDrive = hardwareMap.dcMotor.get("left_forward_drive");
        rightBackDrive = hardwareMap.dcMotor.get("right_back_drive");
        rightForwardDrive = hardwareMap.dcMotor.get("right_forward_drive");
        //servoTest = hardwareMap.servo.get("servo_test");
        //servoTest.setPosition(0);
    }

    public void init_loop() {

    }

    public void start() {
        //servoTest.setPosition(1);
    }

    @Override
    public void loop() {
        //if (first) {
        //        Looper.loop();
        //        first = false;
        //}

        //double val = gyro.getAngle();
        //telemetry.addData("sensor", Double.toString(val));
        //telemetry.addData("Light sensor", lightSensor.getVoltage());

        if (lightSensor.getVoltage() < 1)
        {
            leftBackDrive.setPower(1);
            rightBackDrive.setPower(0);
            leftForwardDrive.setPower(1);
            rightForwardDrive.setPower(0);
        }
        else
        {
            leftBackDrive.setPower(0);
            rightBackDrive.setPower(1);
            leftForwardDrive.setPower(0);
            rightForwardDrive.setPower(1);
        }
    }
//    }
}
