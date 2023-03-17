package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptTensorFlowObjectDetection;

public class HardwareDeTest {

    HardwareMap hmap = null; // configuratia

    public DcMotor leftMotorBack;
    public DcMotor leftMotorFront = null;
    public DcMotor rightMotorBack = null;
    public DcMotor rightMotorFront = null;
    public DcMotor glisieraMotorDreapta = null;
    public DcMotor glisieraMotorStanga = null;

    public DcMotor glisieraMotor = null;
    public NavigationDeTest navigation = null;




    public BNO055IMU imu;


    public ModernRoboticsI2cRangeSensor distance_spate = null;
    public Servo servoGripper = null;
    public Servo servo_pendul_right = null;
    public Servo servo_pendul_left = null;

    public CRServo servo2 = null;


    public ConceptTensorFlowObjectDetection tensorflow = null;
//    public LoggerData loggerData = null;


    public HardwareDeTest(){

    }

    public void init(HardwareMap tmap){

        hmap = tmap;
        navigation = new NavigationDeTest(this);
//        leftMotorFront = hmap.get(DcMotor.class, "left_motor_front");
//
//        leftMotorBack = hmap.get(DcMotor.class, "left_motor_back");
//        rightMotorBack = hmap.get(DcMotor.class, "right_motor_back");
//        rightMotorFront = hmap.get(DcMotor.class, "right_motor_front");

        imu = hmap.get(BNO055IMU.class, "imu");

        glisieraMotorDreapta = hmap.get(DcMotor.class, "glisiera_motor_right");
        glisieraMotorStanga = hmap.get(DcMotor.class, "glisiera_motor_left");
//
        servoGripper = hmap.get(Servo.class, "servo_gripper");
//
        servo_pendul_right = hmap.get(Servo.class, "servo_pendul_right");
        servo_pendul_left = hmap.get(Servo.class, "servo_pendul_left");
//
//        rightMotorBack.setDirection(DcMotor.Direction.REVERSE);
//        rightMotorFront.setDirection(DcMotor.Direction.REVERSE);

        glisieraMotorStanga.setDirection(DcMotor.Direction.REVERSE);

//        leftMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        glisieraMotorStanga.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        glisieraMotorDreapta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        leftMotorBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightMotorBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        leftMotorFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightMotorFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        glisieraMotorStanga.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        glisieraMotorDreapta.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        leftMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightMotorFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        leftMotorFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        glisieraMotorDreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
}