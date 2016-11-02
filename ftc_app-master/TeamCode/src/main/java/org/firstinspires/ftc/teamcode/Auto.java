package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Segway on 29-Oct-16.
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="EmuOp: Autonomous", group="EmuOp")

public class Auto extends OpMode {
    DcMotor leftBackDrive;
    DcMotor rightBackDrive;
    DcMotor leftForwardDrive;
    DcMotor rightForwardDrive;
    DcMotor shooter;
    DcMotor intake;
    Servo buttonPusher;
    String motorType;
    ElapsedTime timer;
    float autoTime;
    int state;
    int INITIAL_STATE;
    int BUTTON_STATE;
    int PARTICLE_STATE;
    int PARK_STATE;

    public void setConstants() {
        INITIAL_STATE = 0;
        BUTTON_STATE = 1;
        PARTICLE_STATE = 3;
        PARK_STATE = 4;
    }
    public void init(){
        setConstants();

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
        buttonPusher = hardwareMap.servo.get("button_pusher");
        motorType = "mech";
        timer = new ElapsedTime();
        autoTime = 2.5f;
        state = 0;
    }

    public void init_start() {
    }

    public void start() {
        timer.reset();
    }

    public void loop() {
        if (state == INITIAL_STATE) {
            if (timer.time() <= autoTime) {
                leftBackDrive.setTargetPosition(1);
                rightBackDrive.setTargetPosition(1);
                leftForwardDrive.setTargetPosition(1);
                rightForwardDrive.setTargetPosition(1);
            } else {
                state = BUTTON_STATE;
            }
        } else if (state == BUTTON_STATE) {
            buttonPusher.setPosition();
        }

    }
}