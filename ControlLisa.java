package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@Config

@TeleOp(name = "Control Lisa", group = "Control")

public class ControlLisa extends LinearOpMode implements OpModeAddition {

    Hardware robot = new Hardware();

    double left;
    double right;
    double speed;
    double direction;

    public static double offsetRight = 0.95;
    public static double sniperSpeed = 1;
    public static boolean sniperEnabled = true;
    String sport = null;
    boolean xHeld = false;

    double servoValue = 0;
    public static double pendulDreaptaPozitieInitiala = 0;
    public static double pendulDreaptaPozititeFinala = 0.75;

    public static double pendulStangaPozitieInitiala = 1 - 0.03;
    public static double pendulStangaPozititeFinala = 0.25 - 0.03;

    public static double pendulStangaIntermediar = 0.6;
    public static double pendulDreaptaIntermediar = 0.4;

    public static double gripperDeschis = 0;
    public static double gripperInchis = 0.5;

    @Override
    public boolean isOpModeIsActive() {
        return opModeIsActive();
    }


    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        robot.servo_pendul_left.setPosition(pendulStangaPozitieInitiala);
        robot.servo_pendul_right.setPosition(pendulDreaptaPozitieInitiala);
        robot.navigation.resetEncoders();
        waitForStart();

        while (opModeIsActive()) {


            speed = gamepad1.left_trigger - gamepad1.right_trigger;
            direction = gamepad1.left_stick_x;


            right = speed + direction;
            left = speed -  direction;

            if (right > 1)
                right = 1;
            if (right < -1)
                right = -1;
            if (left > 1)
                left = 1;
            if (left < -1)
                left = -1;


            if (gamepad1.right_stick_x != 0) {

                robot.leftMotorBack.setPower(gamepad1.right_stick_x * sniperSpeed);
                robot.leftMotorFront.setPower(-gamepad1.right_stick_x * sniperSpeed);
                robot.rightMotorBack.setPower(-gamepad1.right_stick_x * sniperSpeed);
                robot.rightMotorFront.setPower(gamepad1.right_stick_x * sniperSpeed);

            }
            if(gamepad1.right_stick_x == 0){
                robot.leftMotorBack.setPower(left * sniperSpeed);
                robot.leftMotorFront.setPower(left * sniperSpeed);
                robot.rightMotorBack.setPower(right * sniperSpeed);
                robot.rightMotorFront.setPower(right * sniperSpeed);
            }

            if(gamepad2.dpad_down)
            {
                robot.glisieraMotorStanga.setPower(0.4);
                robot.glisieraMotorDreapta.setPower(0.4);
            }
            if(gamepad2.dpad_up) {

                robot.glisieraMotorStanga.setPower(-0.7);
                robot.glisieraMotorDreapta.setPower(-0.7);
            }
            if(!gamepad2.dpad_down && !gamepad2.dpad_up) {
                robot.glisieraMotorStanga.setPower(-0.05);
                robot.glisieraMotorDreapta.setPower(-0.05);
            }



            if(gamepad2.a) {
                robot.servoGripper.setPosition(gripperDeschis);
            }
            else {
                robot.servoGripper.setPosition(gripperInchis);
            }



            // partea de pendul

            if(gamepad2.left_bumper){
                robot.servo_pendul_right.setPosition(pendulDreaptaPozititeFinala);
                robot.servo_pendul_left.setPosition(pendulStangaPozititeFinala);
            }
            if(gamepad2.right_bumper){
                robot.servo_pendul_right.setPosition(pendulDreaptaPozitieInitiala);
                robot.servo_pendul_left.setPosition(pendulStangaPozitieInitiala);
            }
            if(gamepad2.y){

                robot.servo_pendul_right.setPosition(pendulDreaptaIntermediar);
                robot.servo_pendul_left.setPosition(pendulStangaIntermediar);
            }


            if (gamepad1.x) {
                if (!xHeld) {
                    sniperEnabled = !sniperEnabled;
                }
                xHeld = true;
            } else {
                xHeld = false;
            }

            if (sniperEnabled) {
                sniperSpeed = 0.5;
                sport = "dezactivat";
            } else {
                sniperSpeed = 1;
                sport = "activat";

            }



            telemetry.addData("leftBack", robot.leftMotorBack.getCurrentPosition());
             telemetry.addData("leftFront", robot.leftMotorFront.getCurrentPosition());
            telemetry.addData("rightBack", robot.rightMotorBack.getCurrentPosition());
            telemetry.addData("rightFront", robot.rightMotorFront.getCurrentPosition());
            telemetry.addData("Modul sport", sport);
            telemetry.addData("glisieraRight", robot.glisieraMotorDreapta.getCurrentPosition());
            telemetry.addData("glisieraLeft", robot.glisieraMotorStanga.getCurrentPosition());

            telemetry.addData("Servo pendul dreapta",robot.servo_pendul_right.getPosition());
            telemetry.addData("Servo pendul dreapta",robot.servo_pendul_left.getPosition());



            telemetry.update();

        }

    }

}

