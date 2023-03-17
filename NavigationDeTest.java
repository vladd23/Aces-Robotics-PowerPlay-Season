package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;

import java.util.Objects;


public class NavigationDeTest extends LinearOpMode {
    HardwareDeTest robot;
    OpModeAddition opMode;
    Telemetry telemetry;

    public HardwareMap hMap = null;

    double reductie = 13.7; //pt motor 40:1 , 13.7:1 pt gobilda
    double coutPerRev = 1120; //count ul encoderului :)
    double wheelDiam = 96;
    double k = (reductie * coutPerRev) / (wheelDiam * 3.14);

    double GlobalAngle = 0.0;
    Orientation lastAngle = new Orientation();

    public NavigationDeTest(HardwareDeTest robot){
        this.robot = robot;
    }

    public void setOpModeAddition(OpModeAddition opModee){
        this.opMode = opModee;
    }

    public void setHardwareMap(HardwareMap hmap){ this.hMap = hmap; }

    public void setTelemetry(OpMode opMode){
        telemetry = new TelemetryImpl(opMode);
    }

    public void resetEncoders()
    {
        robot.leftMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftMotorFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightMotorFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightMotorBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftMotorBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void imuInit()
    {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();



        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.loggingEnabled = true;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingTag = "IMU";


        robot.imu.initialize(parameters);

        while(opMode.isOpModeIsActive() && !robot.imu.isGyroCalibrated()){
            waitUntil(0.05);
        }

//        telemetry.addData("imu calib status", robot.imu.getCalibrationStatus().toString());
//        telemetry.update();

    }



    public void ResetAngle()
    {


          lastAngle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);

          GlobalAngle = 0;
    }

    public double getAngle()
    {
        Orientation aNgLeS = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);

        double deltaAngle = aNgLeS.firstAngle - lastAngle.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        telemetry.addData("DeltaAngle: ", deltaAngle);


        GlobalAngle += deltaAngle;

        lastAngle = aNgLeS;

        return GlobalAngle;
    }

    public void Turn ( double angle, double speed )
    {

        ResetAngle();
        if (angle < 0)
        {
            robot.leftMotorFront.setPower(-speed);
            robot.leftMotorBack.setPower(-speed);
            robot.rightMotorFront.setPower(speed);
            robot.rightMotorBack.setPower(speed);

            while(opMode.isOpModeIsActive() && getAngle() > angle)
            {

                telemetry.addData("unghi2: ", getAngle());
                telemetry.update();
            }
        }
        if (angle > 0)
        {
            robot.leftMotorFront.setPower(speed);
            robot.leftMotorBack.setPower(speed);
            robot.rightMotorFront.setPower(-speed);
            robot.rightMotorBack.setPower(-speed);

            while(opMode.isOpModeIsActive() && getAngle() <= angle)
            {
                telemetry.addData("unghi1: ", getAngle());
                telemetry.update();

            }
        }

//        frana();
        robot.leftMotorBack.setPower(0);
        robot.leftMotorFront.setPower(0);
        robot.rightMotorBack.setPower(0);
        robot.rightMotorFront.setPower(0);
//        frana();

        waitUntil(0.5);
//        ResetAngle();

    }


    public void frana(){
        robot.leftMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightMotorFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftMotorFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void drive(int Target, double Speed){

        Target = (int)(Target * k);
        if(Target < 0) Speed = Speed*(-1);

        robot.leftMotorBack.setTargetPosition(robot.leftMotorBack.getCurrentPosition() + Target);
        robot.rightMotorBack.setTargetPosition(robot.rightMotorBack.getCurrentPosition() + Target);
        robot.leftMotorFront.setTargetPosition(robot.leftMotorFront.getCurrentPosition() + Target);
        robot.rightMotorFront.setTargetPosition(robot.rightMotorFront.getCurrentPosition() + Target);

        robot.leftMotorFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); /// run to pos
        robot.rightMotorFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftMotorBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightMotorBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        robot.rightMotorBack.setPower(Speed);
        robot.leftMotorBack.setPower(Speed);
        robot.rightMotorFront.setPower(Speed);
        robot.leftMotorFront.setPower(Speed);


        while(robot.leftMotorBack.isBusy() && robot.rightMotorBack.isBusy() && robot.leftMotorFront.isBusy() && robot.rightMotorFront.isBusy() && opMode.isOpModeIsActive()){


            telemetry.addData("right_encoder_front", robot.rightMotorFront.getCurrentPosition());
            telemetry.addData("right_encoder_back", robot.rightMotorBack.getCurrentPosition());
            telemetry.addData("left_motor_back", robot.leftMotorBack.getCurrentPosition());
            telemetry.addData("left_motor_front", robot.leftMotorFront.getCurrentPosition());
            telemetry.addData("target", robot.leftMotorBack.getTargetPosition());
            telemetry.update();
        }
//
//        robot.leftMotorFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // dezactivezi run to pos
//        robot.rightMotorFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        robot.leftMotorBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        robot.rightMotorBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.rightMotorBack.setPower(0);
        robot.leftMotorBack.setPower(0);
        robot.rightMotorFront.setPower(0);
        robot.leftMotorFront.setPower(0);

    }

    public void waitUntil(double waitTime)
    {
        ElapsedTime time = new ElapsedTime();

        while(time.seconds() <= waitTime && opMode.isOpModeIsActive())
        {

        }
    }

    public void SlideOnTime(double time, double speed){
        robot.leftMotorBack.setPower(speed);
        robot.leftMotorFront.setPower(-speed);
        robot.rightMotorBack.setPower(-speed);
        robot.rightMotorFront.setPower(speed);
        waitUntil(time);
        robot.leftMotorFront.setPower(0);
        robot.leftMotorBack.setPower(0);
        robot.rightMotorFront.setPower(0);
        robot.rightMotorBack.setPower(0);

    }

    public void DriveOnTime (double time, double speed)
    {
        robot.leftMotorBack.setPower(-speed);
        robot.leftMotorFront.setPower(-speed);
        robot.rightMotorBack.setPower(-speed);
        robot.rightMotorFront.setPower(-speed);

        waitUntil(time);

        robot.leftMotorFront.setPower(0);
        robot.leftMotorBack.setPower(0);
        robot.rightMotorFront.setPower(0);
        robot.rightMotorBack.setPower(0);

    }

    public void DriveInCm(double distanta){

        double time;
        double distantaInitiala = 183;
        double timeInitial = 1.4;
        time = timeInitial * distanta / distantaInitiala;

        DriveOnTime(time, 0.5);
    }

    public void pusConLowDreapta(Double power){
        robot.navigation.catchCone();
        robot.navigation.waitUntil(1);
        robot.navigation. glisieraNivelLowUpAndDown(2.5,0.8);
        robot.navigation.SlideOnTime(1.5,power);
        robot.navigation.waitUntil(1);
        robot.navigation.DriveOnTime(0.5,power);
        robot.navigation.waitUntil(1);
        robot.navigation. glisieraNivelLowUpAndDown(0.8,-0.8);
        robot.navigation.detachCone();
        robot.navigation.DriveOnTime(0.5,-power);
    }

    public void pusConLowStanga(Double power){
        robot.navigation.catchCone();
        robot.navigation.waitUntil(1);
        robot.navigation. glisieraNivelLowUpAndDown(2.5,-0.8);
        robot.navigation.SlideOnTime(1.5,-power);
        robot.navigation.waitUntil(1);
        robot.navigation.DriveOnTime(0.5,power);
        robot.navigation.waitUntil(1);
        robot.navigation. glisieraNivelLowUpAndDown(0.8,0.8);
        robot.navigation.detachCone();
        robot.navigation.DriveOnTime(0.5,-power);
    }

    public void catchCone(){
        robot.servoGripper.setPosition(0.5);
//        robot.servo2.setPower(-0.4);

    }

    public void detachCone(){
//        robot.servo1.setPower(0);
//        robot.servo2.setPower(0);
    }

    public void pendulFata(){
        robot.servo_pendul_right.setPosition(0);
        robot.servo_pendul_left.setPosition(1);
    }

    public void pendulSpate(){
        robot.servo_pendul_right.setPosition(0.7);
        robot.servo_pendul_left.setPosition(0.3);
    }

    public void glisieraNivelLowUpAndDown(double time, double power){
        robot.glisieraMotor.setPower(power);
        waitUntil(time);
        robot.glisieraMotor.setPower(0);
    }

    public void glisieraPeNivel(String nivel){
        robot.glisieraMotor.setPower(0.8);

        if(Objects.equals(nivel, "low"))
        {
            waitUntil(2.5);

        }
        else if(Objects.equals(nivel, "medium")){
            waitUntil(4.3);
        }
        else if(Objects.equals(nivel, "high")){
            waitUntil(6);
        }

        robot.glisieraMotor.setPower(0) ;
    }

    public void TurnOnTime(double time, double speed){
        robot.leftMotorBack.setPower(speed);
        robot.leftMotorFront.setPower(speed);
        robot.rightMotorBack.setPower(-speed);
        robot.rightMotorFront.setPower(-speed);

        waitUntil(time);

        robot.leftMotorFront.setPower(0);
        robot.leftMotorBack.setPower(0);
        robot.rightMotorFront.setPower(0);
        robot.rightMotorBack.setPower(0);
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }


//    public void DriveToDistance(int dist, double Speed)
//    {
//        if(dist < 0)
//            Speed = Speed * (-1);
//        robot.rightMotorBack.setPower(Speed);
//        robot.leftMotorBack.setPower(Speed);
//        robot.rightMotorFront.setPower(Speed);
//        robot.leftMotorFront.setPower(Speed);
//
//        if(dist > 0) {
//            while(((robot.distance_right.getDistance(DistanceUnit.CM) > dist) || (robot.distance_left.getDistance(DistanceUnit.CM) > dist)) && opMode.isOpModeIsActive()){
//
//            }
//        }
//
//        robot.rightMotorBack.setPower(0);
//        robot.leftMotorBack.setPower(0);
//        robot.rightMotorFront.setPower(0);
//        robot.leftMotorFront.setPower(0);
//
//    }






}