package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 Created by Segway, Vorax and Swacky on 29-Oct-16.
*/


@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="EmuOp: Autonomous", group="EmuOp")

public class Auto extends OpMode {
    DcMotor leftBackDrive;
    DcMotor rightBackDrive;
    DcMotor leftForwardDrive;
    DcMotor rightForwardDrive;
    DcMotor shooter;
    DcMotor intake;
    Servo buttonPivot;
    AnalogInput lightSensor;
    //AnalogInput rightLightSensor;
    AnalogInput redColorSensor;
    AnalogInput blueColorSensor;
    String motorType;
    ElapsedTime timer;
    int team;
    int state;
    DigitalChannel distanceSensor;

    double tileLight;

    int TO_FIRST_LINE;
    int CHECK_FIRST_LINE;
    int TURN_TO_FIRST_BEACON;
    int STRAFE_TO_SECOND_BEACON;
    int CHECK_SECOND_LINE;
    int TURN_TO_SECOND_BEACON;
    int TURN_TO_SHOOT;

    int BEACON_DISTANCE;
    int BUTTON_PUSH_TIME;
    double LIGHT_THRESHOLD;

    int BLUE;
    int RED;
    int LEFT;
    int RIGHT;

    public void setConstants() {
        TO_FIRST_LINE = 0;
        CHECK_FIRST_LINE = 1;
        TURN_TO_FIRST_BEACON = 2;
        STRAFE_TO_SECOND_BEACON = 3;
        CHECK_SECOND_LINE = 4;
        TURN_TO_SECOND_BEACON = 5;
        TURN_TO_SHOOT = 6;

        BLUE = 0;
        RED = 1;
        LEFT = 0;
        RIGHT = 1;
        BEACON_DISTANCE = 500;
        BUTTON_PUSH_TIME = 1000;
        LIGHT_THRESHOLD = 0.5;
    }
    public void init(){
        setConstants();
        team = BLUE;

        leftBackDrive = hardwareMap.dcMotor.get("left_back_drive");
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive = hardwareMap.dcMotor.get("right_back_drive");
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftForwardDrive = hardwareMap.dcMotor.get("left_forward_drive");
        leftForwardDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightForwardDrive = hardwareMap.dcMotor.get("right_forward_drive");
        rightForwardDrive.setDirection(DcMotor.Direction.REVERSE);
        rightForwardDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter = hardwareMap.dcMotor.get("shooter");
        buttonPivot = hardwareMap.servo.get("button_pivot");
        redColorSensor = hardwareMap.analogInput.get("red_color_sensor");
        blueColorSensor = hardwareMap.analogInput.get("blue_color_sensor");
        lightSensor = hardwareMap.analogInput.get("light_sensor");
        distanceSensor = hardwareMap.digitalChannel.get("distance_sensor");

        motorType = "mech";
        timer = new ElapsedTime();
        state = TO_FIRST_LINE;
    }

    public void init_start() {
        tileLight = lightSensor.getVoltage();
    }

    public void start() {
        timer.reset();
        state = TO_FIRST_LINE;
    }

    public void loop() {
        telemetry.addData("State",state);
        if (state == TO_FIRST_LINE) {
            leftBackDrive.setPower(1);
            rightBackDrive.setPower(1);
            leftForwardDrive.setPower(1);
            rightForwardDrive.setPower(1);
            state = CHECK_FIRST_LINE;
        } else if (state == CHECK_FIRST_LINE) {
            if (checkIfLine()) {
                leftBackDrive.setPower(0);
                rightBackDrive.setPower(0);
                leftForwardDrive.setPower(0);
                rightForwardDrive.setPower(0);
                state = TURN_TO_FIRST_BEACON;
            }
        } else if (state == TURN_TO_FIRST_BEACON) {
            if (distanceSensor.getState()) {
                if (checkIfLine()) {
                    leftBackDrive.setPower(0.3);
                    rightBackDrive.setPower(0);
                    leftForwardDrive.setPower(0.3);
                    rightForwardDrive.setPower(0);
                } else {
                    leftBackDrive.setPower(0);
                    rightBackDrive.setPower(0.3);
                    leftForwardDrive.setPower(0);
                    rightForwardDrive.setPower(0.3);
                }
            } else {
                leftBackDrive.setPower(0);
                rightBackDrive.setPower(0);
                leftForwardDrive.setPower(0);
                rightForwardDrive.setPower(0);
                pushButton(findBeaconSide());
                state = STRAFE_TO_SECOND_BEACON;
            }
        } else if (state == STRAFE_TO_SECOND_BEACON) {
            if (team == BLUE) {
                leftBackDrive.setPower(1);
                rightBackDrive.setPower(-1);
                leftForwardDrive.setPower(-1);
                rightForwardDrive.setPower(1);
            } else {
                leftBackDrive.setPower(-1);
                rightBackDrive.setPower(1);
                leftForwardDrive.setPower(1);
                rightForwardDrive.setPower(-1);
            }
            state = CHECK_SECOND_LINE;
        } else if (state == CHECK_SECOND_LINE) {
            if (checkIfLine()) {
                leftBackDrive.setPower(0);
                rightBackDrive.setPower(0);
                leftForwardDrive.setPower(0);
                rightForwardDrive.setPower(0);
                state = TURN_TO_SECOND_BEACON;
            }
        } else if (state == TURN_TO_SECOND_BEACON) {
            if (distanceSensor.getState()) {
                if (checkIfLine()) {
                    leftBackDrive.setPower(0.3);
                    rightBackDrive.setPower(0);
                    leftForwardDrive.setPower(0.3);
                    rightForwardDrive.setPower(0);
                } else {
                    leftBackDrive.setPower(0);
                    rightBackDrive.setPower(0.3);
                    leftForwardDrive.setPower(0);
                    rightForwardDrive.setPower(0.3);
                }
            } else {
                leftBackDrive.setPower(0);
                rightBackDrive.setPower(0);
                leftForwardDrive.setPower(0);
                rightForwardDrive.setPower(0);
                pushButton(findBeaconSide());
                state = TURN_TO_SHOOT;
            }
        } else if (state == TURN_TO_SHOOT) {
            //...
        }
    }

    public int findBeaconSide() {
        buttonPivot.setPosition(0.2);
        if (redColorSensor.getVoltage() < blueColorSensor.getVoltage()) {
            //Left is more red, right is more blue
            if (team == RED) {
                return LEFT;
            } else {
                return RIGHT;
            }
        } else {
            //Left is more blue, right is more red
            if (team == BLUE) {
                return LEFT;
            } else {
                return RIGHT;
            }
        }
    }

    public void pushButton(int side) {
        buttonPivot.setPosition(side);
        while (Math.abs(buttonPivot.getPosition() - side) > 0.1) {

        }
        leftBackDrive.setPower(0.2);
        rightBackDrive.setPower(0.2);
        leftForwardDrive.setPower(0.2);
        rightForwardDrive.setPower(0.2);
        try {
            timer.wait(BUTTON_PUSH_TIME);
        } catch (InterruptedException e) {
            telemetry.addData("Interrupted", e);
        }
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftForwardDrive.setPower(0);
        rightForwardDrive.setPower(0);
    }

    public boolean checkIfLine() {
        return Math.abs(lightSensor.getVoltage() - tileLight) > LIGHT_THRESHOLD;
    }
}