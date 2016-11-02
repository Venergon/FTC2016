package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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
    DcMotor leftShooter;
    DcMotor rightShooter;
    DcMotor intake;
    Servo buttonPusher;
    String motorType;
    ElapsedTime timer;
    float autoTime;

    public void init(){
        leftBackDrive = hardwareMap.dcMotor.get("left_back_drive");
        rightBackDrive = hardwareMap.dcMotor.get("right_back_drive");
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftForwardDrive = hardwareMap.dcMotor.get("left_forward_drive");
        rightForwardDrive = hardwareMap.dcMotor.get("right_forward_drive");
        rightForwardDrive.setDirection(DcMotor.Direction.REVERSE);
        motorType = "mech";
        timer = new ElapsedTime();
        autoTime = 2.5f;
    }

    public void init_start() {
    }

    public void start() {
        timer.reset();
    }

    public void loop() {
        if (timer.time() <= autoTime) {
            leftBackDrive.setPower(-1);
            rightBackDrive.setPower(-1);
            leftForwardDrive.setPower(-1);
            rightForwardDrive.setPower(-1);
        }
        else {
            leftBackDrive.setPower(0);
            rightBackDrive.setPower(0);
            leftForwardDrive.setPower(0);
            rightForwardDrive.setPower(0);
        }
    }
}