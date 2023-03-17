package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptTensorFlowObjectDetection;

public class Hardware {


    HardwareMap hmap = null;

    public DcMotorEx leftMotorBack = null;
    public DcMotorEx leftMotorFront = null;
    public DcMotorEx rightMotorBack = null;
    public DcMotorEx rightMotorFront = null;
    public DcMotorEx glisieraMotorDreapta = null;
    public DcMotorEx glisieraMotorStanga = null;

    public DcMotor glisieraMotor = null;
    public Navigation navigation = null;




    public IMU imu;


    public ModernRoboticsI2cRangeSensor distance_spate = null;
    public Servo servoGripper = null;
    public Servo servo_pendul_right = null;
    public Servo servo_pendul_left = null;

    public CRServo servo2 = null;


    public ConceptTensorFlowObjectDetection tensorflow = null;
//    public LoggerData loggerData = null;


    public Hardware(){

    }

    public void init(HardwareMap tmap){

        hmap = tmap;
        navigation = new Navigation(this);
        leftMotorFront = hmap.get(DcMotorEx.class, "left_motor_front");
        leftMotorBack = hmap.get(DcMotorEx.class, "left_motor_back");
        rightMotorBack = hmap.get(DcMotorEx.class, "right_motor_back");
        rightMotorFront = hmap.get(DcMotorEx.class, "right_motor_front");

        imu = hmap.get(IMU.class, "imu");

       glisieraMotorDreapta = hmap.get(DcMotorEx.class, "glisiera_motor_right");
        glisieraMotorStanga = hmap.get(DcMotorEx.class, "glisiera_motor_left");

        servoGripper = hmap.get(Servo.class, "servo_gripper");

        servo_pendul_right = hmap.get(Servo.class, "servo_pendul_right");
        servo_pendul_left = hmap.get(Servo.class, "servo_pendul_left");

        rightMotorBack.setDirection(DcMotorEx.Direction.REVERSE);
        rightMotorFront.setDirection(DcMotorEx.Direction.REVERSE);

        glisieraMotorStanga.setDirection(DcMotorEx.Direction.REVERSE);

        leftMotorBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftMotorFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        glisieraMotorStanga.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        glisieraMotorDreapta.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        glisieraMotorStanga.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        glisieraMotorDreapta.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        leftMotorBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightMotorBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightMotorFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftMotorFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        glisieraMotorDreapta.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        glisieraMotorStanga.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

    }
}