package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import java.security.KeyStore;
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
    DcMotor rightShooter;
    DcMotor intake;
    Servo buttonPusher;
    String motorType;
    float joystick_1_x;
    float joystick_1_y;
    float joystick_2_x;
    float joystick_2_y;
    boolean buttonX;
    boolean buttonB;
    boolean buttonY;
    boolean buttonA;
    //Run once when turned on
    @Override
    public void init() {
        leftBackDrive = hardwareMap.dcMotor.get("left_back_drive");
        rightBackDrive = hardwareMap.dcMotor.get("right_back_drive");
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftForwardDrive = hardwareMap.dcMotor.get("left_forward_drive");
        rightForwardDrive = hardwareMap.dcMotor.get("right_forward_drive");
        rightForwardDrive.setDirection(DcMotor.Direction.REVERSE);
        shooter = hardwareMap.dcMotor.get("shooter");
        //rightShooter = hardwareMap.dcMotor.get("right_shooter");
        //intake = hardwareMap.dcMotor.get("intake");
        buttonPusher = hardwareMap.servo.get("buttonPusher");
        motorType = "mech";
        joystick_1_x = 0;
        joystick_1_y = 0;
        joystick_2_x = 0;
        joystick_2_y = 0;
        buttonX = false;
        buttonB = false;
        buttonY = false;
        buttonA = false;
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

        //update motors
        updateMotors(gamepad1);

        //update shooters
        updateShooters(buttonX);

        //update intake
        //updateIntake(buttonB);

        //update button pusher
        updateButtonPusher(buttonY);
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
        updateShooters(false);
        //updateIntake(false);
        updateButtonPusher(false);
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
            float gply = gamepad.left_stick_y;
            float gplx = gamepad.left_stick_x;
            float gprx = gamepad.right_stick_x;
            float lf = gply+gplx+gprx;
            float rf = gply-gplx-gprx;
            float lb = gply-gplx+gprx;
            float rb = gply+gplx-gprx;
            float sortList[] = {Math.abs(lf),Math.abs(rf),Math.abs(lb),Math.abs(rb)};
            Arrays.sort(sortList);
            if (sortList[3] != 0) {
                lf = lf / sortList[3];
                rf = rf / sortList[3];
                lb = lb / sortList[3];
                rb = rb / sortList[3];
            }
            leftBackDrive.setPower(lb);
            rightBackDrive.setPower(rb);
            leftForwardDrive.setPower(lf);
            rightForwardDrive.setPower(rf);
        }
    }

    public void updateShooters(boolean shouldShoot) {
        if (shouldShoot) {
            shooter.setPower(1);
            //rightShooter.setPower(-1);
        } else {
            shooter.setPower(0);
            //rightShooter.setPower(0);
        }
    }

    public void updateIntake(boolean shouldIntake) {
        if (shouldIntake) {
            intake.setPower(1);
        } else {
            intake.setPower(0);
        }
    }

    public void updateButtonPusher(boolean shouldPress) {
        if (shouldPress) {
            buttonPusher.setPosition(90);
        } else {
            buttonPusher.setPosition(0);
        }
    }
}
