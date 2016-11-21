//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.AnalogInput;
//import com.qualcomm.robotcore.hardware.ColorSensor;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.LightSensor;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.UltrasonicSensor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
///**
// * Created by Segway on 29-Oct-16.
// */
//
//@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="EmuOp: Autonomous", group="EmuOp")
//
//public class Auto extends OpMode {
//    DcMotor leftBackDrive;
//    DcMotor rightBackDrive;
//    DcMotor leftForwardDrive;
//    DcMotor rightForwardDrive;
//    //DcMotor shooter;
//    //DcMotor intake;
//    //Servo buttonPusher;
//    //AnalogInput leftLightSensor;
////    AnalogInput rightLightSensor;
//    //AnalogInput leftColorSensor;
//    //AnalogInput rightColorSensor;
//    String motorType;
//    ElapsedTime timer;
//    int team;
//    int state;
//    int TO_FIRST_LINE;
//    int CHECK_FIRST_LINE;
//    int TURN_TO_FIRST_BEACON;
//    int FIND_WHICH_SIDE_FIRST;
//    int PUSH_FIRST_BEACON;
//    int TURN_TO_SECOND_LINE;
//    int TO_SECOND_LINE;
//    int TURN_TO_SECOND_BEACON;
//    int FIND_WHICH_SIDE_SECOND;
//    int PUSH_SECOND_BEACON;
//    int BEACON_DISTANCE;
//    int LINE_VALUE;
//    UltrasonicSensor distanceSensor;
//
//    int BLUE;
//    int RED;
//
//    public void setConstants() {
//        TO_FIRST_LINE = 0;
//        CHECK_FIRST_LINE = 1;
//        TURN_TO_FIRST_BEACON = 2;
//        FIND_WHICH_SIDE_FIRST = 3;
//        PUSH_FIRST_BEACON = 4;
//        TURN_TO_SECOND_LINE = 5;
//        TO_SECOND_LINE = 6;
//        TURN_TO_SECOND_BEACON = 7;
//        FIND_WHICH_SIDE_SECOND = 8;
//        PUSH_SECOND_BEACON = 9;
//
//        BLUE = 0;
//        RED = 1;
//        BEACON_DISTANCE = 500;
//        LINE_VALUE = 300;
//    }
//    public void init(){
//        setConstants();
//        team = BLUE;
//
//        leftBackDrive = hardwareMap.dcMotor.get("left_back_drive");
//        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightBackDrive = hardwareMap.dcMotor.get("right_back_drive");
//        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
//        leftForwardDrive = hardwareMap.dcMotor.get("left_forward_drive");
//        leftForwardDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightForwardDrive = hardwareMap.dcMotor.get("right_forward_drive");
//        rightForwardDrive.setDirection(DcMotor.Direction.REVERSE);
//        rightForwardDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        //shooter = hardwareMap.dcMotor.get("shooter");
//        //buttonPusher = hardwareMap.servo.get("button_pusher");
//        //leftLightSensor = hardwareMap.analogInput.get("left_light_sensor");
//        //rightLightSensor = hardwareMap.analogInput.get("right_light_sensor");
////        leftColorSensor = hardwareMap.analogInput.get("left_color_sensor");
////        rightColorSensor = hardwareMap.analogInput.get("right_color_sensor");
//        distanceSensor = hardwareMap.ultrasonicSensor.get("distanceSensor");
//
//        motorType = "mech";
//        timer = new ElapsedTime();
//        //autoTime = 2.5f;
//        state = TO_FIRST_LINE;
//    }
//
//    public void init_start() {
//
//    }
//
//    public void start() {
//        timer.reset();
//        state = TO_FIRST_LINE;
//    }
//
//    public void loop() {
//        if (state == TO_FIRST_LINE) {
//            leftBackDrive.setPower(1);
//            rightBackDrive.setPower(1);
//            leftForwardDrive.setPower(1);
//            rightForwardDrive.setPower(1);
//            state = CHECK_FIRST_LINE;
//        } else if (state == CHECK_FIRST_LINE) {
//            if (team == BLUE) {
//                if (rightLightSensor.getVoltage() < LINE_VALUE) {
//                    leftBackDrive.setPower(0);
//                    rightBackDrive.setPower(0);
//                    leftForwardDrive.setPower(0);
//                    rightForwardDrive.setPower(0);
//                    state = TURN_TO_FIRST_BEACON;
//                }
//            } else {
//                //...
//                if (leftLightSensor.getVoltage() < LINE_VALUE) {
//                    leftBackDrive.setPower(0);
//                    rightBackDrive.setPower(0);
//                    leftForwardDrive.setPower(0);
//                    rightForwardDrive.setPower(0);
//                    state = TURN_TO_FIRST_BEACON;
//                }
//
//            }
//        } else if (state==TURN_TO_FIRST_BEACON) {
//            if (distanceSensor.getUltrasonicLevel() < BEACON_DISTANCE) {
//                leftBackDrive.setPower(0);
//                rightBackDrive.setPower(0);
//                leftForwardDrive.setPower(0);
//                rightForwardDrive.setPower(0);
//                state = TURN_TO_FIRST_BEACON;
//
//            } else if (leftLightSensor.getVoltage() < LINE_VALUE) {
//                leftBackDrive.setPower(0.5);
//                rightBackDrive.setPower(1);
//                leftForwardDrive.setPower(0.5);
//                rightForwardDrive.setPower(1);
//
//            } else if (rightLightSensor.getVoltage() < LINE_VALUE) {
//                rightBackDrive.setPower(0.5);
//                leftBackDrive.setPower(1);
//                rightForwardDrive.setPower(0.5);
//                leftForwardDrive.setPower(1);
//
//            } else {
//                rightBackDrive.setPower(1);
//                leftBackDrive.setPower(1);
//                rightForwardDrive.setPower(1);
//                leftForwardDrive.setPower(1);
//            }
//
//        } else if (state == FIND_WHICH_SIDE_FIRST || state == FIND_WHICH_SIDE_SECOND) {
//            if (leftColorSensor.getVoltage() < rightColorSensor.getVoltage()) {
//                // Left sensor more red
//                telemetry.addData("Left is more","red");
//            } else {
//                telemetry.addData("Left is more","blue");
//            }
//        }
//
//    }
//
//
//}