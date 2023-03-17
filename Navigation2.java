package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Objects;

public class Navigation2 extends LinearOpMode {
    Hardware robot;

    private static double ticksPerRev = (1.0 + (46.0 / 17.0)) * (1.0 + (46.0 / 17.0)) * 28.0;
    private static double mmPerRev = Math.PI * 96;

    public static int mmToTicks(double distance) {
        return (int) (distance * ticksPerRev / mmPerRev);
    }

    DcMotorEx rightBack;
    DcMotorEx rightFront;
    DcMotorEx leftBack;
    DcMotorEx leftFront;

    LinearOpMode opMode;

    public int fowardPosition() {
        return (robot.rightMotorBack.getCurrentPosition() + robot.leftMotorBack.getCurrentPosition()) / 2;
    }

    public double fowardVelocity() {
        return (robot.rightMotorBack.getVelocity() + robot.leftMotorBack.getVelocity()) / 2;
    }

    public Navigation2 (HardwareMap hardwareMap, LinearOpMode opMode) {
        this.opMode = opMode;
        robot = new Hardware();

        /*leftFront = hardwareMap.get(DcMotorEx.class, "left_motor_front");
        leftBack = hardwareMap.get(DcMotorEx.class, "left_motor_back");
        rightBack = hardwareMap.get(DcMotorEx.class, "right_motor_back");
        rightFront = hardwareMap.get(DcMotorEx.class, "right_motor_front");

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); */
    }

    public void foward(double distanceMM) {
        int distance = mmToTicks(distanceMM);
        int target = fowardPosition() + distance;
        double power = 0;

        while ((Math.abs(target - fowardPosition()) > mmToTicks(5) || Math.abs(fowardVelocity()) > mmToTicks(5)) && opMode.opModeIsActive()) {
            if (Math.abs(target - fowardPosition()) < mmToTicks(300))
                power = (target - fowardPosition()) / (float) (mmToTicks(300)) * 0.5;
            else power = Math.signum(target - fowardPosition()) * 0.5;
            robot.leftMotorFront.setPower(power);
            robot.leftMotorBack.setPower(power);
            robot.rightMotorBack.setPower(power * 0.8);
            robot.rightMotorFront.setPower(power * 0.8);
        }

        robot.leftMotorFront.setPower(0);
        robot.leftMotorBack.setPower(0);
        robot.rightMotorBack.setPower(0);
        robot.rightMotorFront.setPower(0);
    }

    public void turn(double distanceMM) {
        int distance = mmToTicks(distanceMM);
        int target = fowardPosition() + distance;
        double power = 0;

        while ((Math.abs(target - fowardPosition()) > mmToTicks(5) || Math.abs(fowardVelocity()) > mmToTicks(5)) && opMode.opModeIsActive()) {
            if (Math.abs(target - fowardPosition()) < mmToTicks(300))
                power = (target - fowardPosition()) / (float) (mmToTicks(300)) * 0.5;
            else power = Math.signum(target - fowardPosition()) * 0.5;
            robot.leftMotorFront.setPower(-power);
            robot.leftMotorBack.setPower(-power);
            robot.rightMotorBack.setPower(power * 0.8);
            robot.rightMotorFront.setPower(power * 0.8);
        }

        robot.leftMotorFront.setPower(0);
        robot.leftMotorBack.setPower(0);
        robot.rightMotorBack.setPower(0);
        robot.rightMotorFront.setPower(0);
    }

    public void catchCone(){
        robot.servoGripper.setPosition(0.5);
    }

    public void detachCone(){
        robot.servoGripper.setPosition(0);
    }

    public void pendulFata(){
        robot.servo_pendul_right.setPosition(0);
        robot.servo_pendul_left.setPosition(1);
    }

    public void pendulSpate(){
        robot.servo_pendul_right.setPosition(0.7);
        robot.servo_pendul_left.setPosition(0.3);
    }

    public void waitUntil(double waitTime)
    {
        ElapsedTime time = new ElapsedTime();

        while(time.seconds() <= waitTime)
        {

        }
    }

    public void glisieraNivelLowUpAndDown(double time, double power){
        robot.glisieraMotor.setPower(power);
        waitUntil(time);
        robot.glisieraMotor.setPower(0);
    }

    public void glisieraPeNivel(String nivel){
        robot.glisieraMotorDreapta.setPower(-0.7);
        robot.glisieraMotorStanga.setPower(-0.7);

        if(Objects.equals(nivel, "low"))
        {
            waitUntil(1);

        }
        else if(Objects.equals(nivel, "medium")){
            waitUntil(3);
        }
        else if(Objects.equals(nivel, "high")){
            waitUntil(4.3);
        }


        robot.glisieraMotorDreapta.setPower(-0.02);
        robot.glisieraMotorStanga.setPower(-0.02);
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }
}
