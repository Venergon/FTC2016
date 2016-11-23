package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorManager;
import android.os.Looper;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Arrays;

/**
 * Created by Segway on 19-Oct-16.
 */

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="EmuOp: Telop Tank", group="EmuOp")

public class TeleOp extends OpMode {
    DcMotor leftBackDrive;
    DcMotor rightBackDrive;
    DcMotor leftForwardDrive;
    DcMotor rightForwardDrive;
    DcMotor shooter;
    //DcMotor rightShooter;
    //DcMotor intake;
    Servo buttonPusherLeft;
    Servo buttonPusherRight;
	MyGyro gyro;
    SensorManager sensorManager;
    String motorType;
    float joystick_1_x;
    float joystick_1_y;
    float joystick_2_x;
    float joystick_2_y;
    boolean buttonX;
    boolean buttonB;
    boolean buttonY;
    boolean buttonA;
    float leftTrigger;
    float rightTrigger;
    boolean leftBumper;
    boolean rightBumper;
    double WEIGHT;
    //Run once when turned on
    @Override
    public void init() {
        leftBackDrive = hardwareMap.dcMotor.get("left_back_drive");
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive = hardwareMap.dcMotor.get("right_back_drive");
        leftForwardDrive = hardwareMap.dcMotor.get("left_forward_drive");
        leftForwardDrive.setDirection(DcMotor.Direction.REVERSE);
        rightForwardDrive = hardwareMap.dcMotor.get("right_forward_drive");
        shooter = hardwareMap.dcMotor.get("shooter");
        //intake = hardwareMap.dcMotor.get("intake");
        buttonPusherLeft = hardwareMap.servo.get("button_pusher_left");
        buttonPusherRight = hardwareMap.servo.get("button_pusher_right");
        sensorManager = (SensorManager)hardwareMap.appContext.getSystemService(Context.SENSOR_SERVICE);
        Looper.prepare();
		gyro =  new MyGyro();
        sensorManager.registerListener(gyro, sensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR), SensorManager.SENSOR_DELAY_GAME);
        motorType = "mech";
        joystick_1_x = 0;
        joystick_1_y = 0;
        joystick_2_x = 0;
        joystick_2_y = 0;
        buttonX = false;
        buttonB = false;
        buttonY = false;
        buttonA = false;
        WEIGHT = 1;
    }

    //Runs repeatedly until play is hit
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */

    @Override
    public void loop() {
        //update input variables
        joystick_1_x = -gamepad1.left_stick_x;
        joystick_1_y = -gamepad1.left_stick_y;
        joystick_2_x = -gamepad1.right_stick_x;
        joystick_2_y = -gamepad1.right_stick_y;
        buttonX = gamepad2.x;
        buttonB = gamepad2.b;
        buttonY = gamepad2.y;
        buttonA = gamepad2.a;
        leftTrigger = gamepad2.left_trigger;
        rightTrigger = gamepad2.right_trigger;
        leftBumper = gamepad2.left_bumper;
        rightBumper = gamepad2.right_bumper;

        //update motors
        updateMotors(gamepad1);

        //update shooters
        updateShooter(buttonX);

        //update intake
        //updateIntake(buttonB);

        //update button pusher
        updateButtonPusher(leftTrigger > 0.5, rightTrigger > 0.5);
    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftForwardDrive.setPower(0);
        rightForwardDrive.setPower(0);
        //updateIntake(false);
        updateButtonPusher(false, false);
    }

    public void updateMotors(Gamepad gamepad) {
        if (motorType.equals("tank")) {
            leftBackDrive.setPower(gamepad.left_stick_y);
            rightBackDrive.setPower(gamepad.right_stick_y);
            leftForwardDrive.setPower(gamepad.left_stick_y);
            rightForwardDrive.setPower(gamepad.right_stick_y);
        } else if (motorType.equals("arcade")) {
            //Coming soon...
        } else if (motorType.equals("mech")) {
			double error = gyro.getAngle();
            telemetry.addData("error", Double.toString(error));
            float gply = -gamepad.left_stick_y;
            float gplx = gamepad.left_stick_x;
            float gprx = gamepad.right_stick_x;
			double turn = gprx;//-error;
            double lf = gply+gplx+turn;
            double rf = gply-gplx-turn;
            double lb = gply-gplx+turn;
            double rb = gply+gplx-turn;
            double sortList[] = {Math.abs(lf),Math.abs(rf),Math.abs(lb),Math.abs(rb)};
            Arrays.sort(sortList);
            if (sortList[3] != 0) {
                lf = lf / sortList[3];
                rf = rf / sortList[3];
                lb = lb / sortList[3];
                rb = rb / sortList[3];
            }
            if (WEIGHT < 1) {
                WEIGHT = 1 / WEIGHT;
                leftBackDrive.setPower(lb);
                rightBackDrive.setPower(rb);
                leftForwardDrive.setPower(lf / WEIGHT);
                rightForwardDrive.setPower(lf / WEIGHT);
            } else {
                leftBackDrive.setPower(lb / WEIGHT);
                rightBackDrive.setPower(rb / WEIGHT);
                leftForwardDrive.setPower(lf);
                rightForwardDrive.setPower(rf);
            }
        }
    }

    public void updateShooter(boolean shouldShoot) {
        if (shouldShoot) {
            shooter.setPower(1);
        } else {
            shooter.setPower(0);
        }
    }

    public void updateIntake(boolean shouldIntake) {
        if (shouldIntake) {
            //intake.setPower(1);
        } else {
            //intake.setPower(0);
        }
    }

    public void updateButtonPusher(boolean trigger1, boolean trigger2) {
        if (trigger1) {
            buttonPusherLeft.setPosition(1);
        } else {
            buttonPusherLeft.setPosition(0);
        }
        if (trigger2) {
            buttonPusherRight.setPosition(1);
        } else {
            buttonPusherRight.setPosition(0);
        }
    }
}