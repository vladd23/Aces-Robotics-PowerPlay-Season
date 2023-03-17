package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Config
@Autonomous(name="AutoStangaBun", group="Pushbot")
public class AutonomieStateMachine extends LinearOpMode {
    DcMotorEx leftMotorBack;
    DcMotorEx leftMotorFront;
    DcMotorEx rightMotorBack;
    DcMotorEx rightMotorFront;

    Hardware robot = new Hardware();
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime runTime = new ElapsedTime();

    int foward = Navigation2.mmToTicks(700);
    int strafe = Navigation2.mmToTicks(350);

    public STATE state = STATE.STRAFE_DE_INCEPUT;

    private int VAR_EDITABLE = 45;
    private double POWER = 0.3;

    public enum STATE {
        STRAFE_DE_INCEPUT,
        WAIT_0,
        FORWARD_1,
        WAIT_1,
        TURN_1,
        WAIT_2,
        FORWARD_2,
        WAIT_3,
        WAIT_4,
        WAIT_5,
        WAIT_6,
        BACK_3,
        TURN_3,
        FORWARD_4,
        WAIT_7,
        TURN_7,
        WAIT_8,
        FORWARD_IN_STACK,
        PUTIN_BACK_DIN_STACK,
        WAIT_9,
        WAIT_10,
        WAIT_11,
        ALLIGN_LOW,
        MAI_IN_FATA_LA_LOW,
        WAIT_12,
        TURN_8,
        DRIVE_BACK,
        WAIT_13,
        WAIT_14, DRIVE_LA_STACK, WAIT_15, SPRE_STACK, DRIVE_BACK2, WAIT_16, TURN_LA_HIGH, ALLIGN_HIGH, WAIT_17, WAIT_18, TURN_LA_LOW, ALLIGN_LOW2, STOP
    }

    public void resetEncoders() {
        robot.leftMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void runToPosition() {
        robot.leftMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setPower(double power) {
        robot.leftMotorBack.setPower(power);
        robot.leftMotorFront.setPower(power);
        robot.rightMotorBack.setPower(power);
        robot.rightMotorFront.setPower(power);
    }

    public void setPowerTurnLeft(double power) {
        robot.leftMotorBack.setPower(power);
        robot.leftMotorFront.setPower(power);
        robot.rightMotorBack.setPower(-power);
        robot.rightMotorFront.setPower(-power);
    }

    public void setPowerTurnRight(double power) {
        robot.leftMotorBack.setPower(-power);
        robot.leftMotorFront.setPower(-power);
        robot.rightMotorBack.setPower(power);
        robot.rightMotorFront.setPower(power);
    }

    public void setTargetFoward(int target) {
        robot.leftMotorBack.setTargetPosition(target);
        robot.leftMotorFront.setTargetPosition(target);
        robot.rightMotorBack.setTargetPosition(target);
        robot.rightMotorFront.setTargetPosition(target);
    }

    public void setTargetStrafeRight(int target) {
        robot.leftMotorBack.setTargetPosition(-target);
        robot.leftMotorFront.setTargetPosition(target);
        robot.rightMotorBack.setTargetPosition(target);
        robot.rightMotorFront.setTargetPosition(-target);
    }

    public void setTargetStrafeLeft(int target) {
        robot.leftMotorBack.setTargetPosition(target);
        robot.leftMotorFront.setTargetPosition(-target);
        robot.rightMotorBack.setTargetPosition(-target);
        robot.rightMotorFront.setTargetPosition(target);
    }

    public void setTargetTurnLeft(int target) {
        robot.leftMotorBack.setTargetPosition(-target);
        robot.leftMotorFront.setTargetPosition(-target);
        robot.rightMotorBack.setTargetPosition(target);
        robot.rightMotorFront.setTargetPosition(target);
    }

    public void setTargetTurnRight(int target) {
        robot.leftMotorBack.setTargetPosition(target);
        robot.leftMotorFront.setTargetPosition(target);
        robot.rightMotorBack.setTargetPosition(-target);
        robot.rightMotorFront.setTargetPosition(-target);
    }

    public void setPowerGlisiere(double power) {
        robot.glisieraMotorStanga.setPower(power);
        robot.glisieraMotorDreapta.setPower(power);
    }

    public boolean isBusy() {
        return robot.leftMotorBack.isBusy() && robot.leftMotorFront.isBusy() && robot.rightMotorBack.isBusy() && robot.rightMotorFront.isBusy();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.init(hardwareMap);
        runTime = new ElapsedTime();
//        robot.navigation.imuInit();

        resetEncoders();

        robot.leftMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftMotorFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightMotorFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.leftMotorBack.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.leftMotorFront.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.rightMotorBack.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.rightMotorFront.setDirection(DcMotorSimple.Direction.FORWARD);

        robot.glisieraMotorDreapta.setTargetPosition(-1800);
        robot.glisieraMotorStanga.setTargetPosition(-1800);
        robot.glisieraMotorDreapta.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.glisieraMotorStanga.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//        setTargetStrafeLeft(Navigation2.mmToTicks(100));
        setTargetFoward(0);
        setPower(0);
        runToPosition();

        robot.servoGripper.setPosition(ControlLisa.gripperInchis);
        robot.servo_pendul_left.setPosition(ControlLisa.pendulStangaPozitieInitiala);
        robot.servo_pendul_right.setPosition(ControlLisa.pendulDreaptaPozitieInitiala);

        waitForStart();
        setPower(0.3);
        setPowerGlisiere(0.6);

        while (opModeIsActive()) {
            switch (state) {
                case STRAFE_DE_INCEPUT:
                    if(!isBusy()) {
                        state = STATE.WAIT_0;
                        setPower(0);
                        resetEncoders();
                        timer.reset();
                    }
                    break;
                case WAIT_0:
                    if(timer.seconds() > 0.05) {
                        state = STATE.FORWARD_1;
                        setTargetFoward(Navigation2.mmToTicks(700));
                        runToPosition();
                        setPower(0.4);
                    }
                    break;
                case FORWARD_1:
                    if (!isBusy()) {
                        state = STATE.WAIT_1;
                        setPower(0);
                        resetEncoders();
                        timer.reset();
                    }
                    break;
                case WAIT_1:
                    // turn pt mid
                    if (timer.seconds() > 0.05) {
                        state = STATE.TURN_1;
                        setTargetTurnRight(345);
                        runToPosition();
                        setPower(0.3);
                    }
                    break;
                case TURN_1:
                    if(!isBusy()) {
                        state = STATE.WAIT_2;
                        setPower(0);
                        resetEncoders();
                        timer.reset();
                    }
                    break;
                case WAIT_2:
                    //pozitionare mid
                    if(timer.seconds() > 0.05) {
                        state = STATE.FORWARD_2;
                        setTargetFoward(Navigation2.mmToTicks(250));
                        runToPosition();
                        setPower(0.4);
                    }
                    break;
                case FORWARD_2:
                    if(!isBusy()) {
                        state = STATE.WAIT_3;
                        setPower(0);
                        resetEncoders();
                        timer.reset();
                    }
                    break;
                case WAIT_3:
                    // pune conu pe mid
                    if(timer.seconds() > 0.05) {
                        robot.servoGripper.setPosition(ControlLisa.gripperDeschis);
                        state = STATE.BACK_3;
                        setTargetFoward(-Navigation2.mmToTicks(250));
                        runToPosition();
                        setPower(0.3);
                    }
                    break;
                case BACK_3:
                    if(!isBusy()) {
                        state = STATE.WAIT_4;
                        setPower(0);
                        resetEncoders();
                        timer.reset();
                    }
                    break;
                case WAIT_4:
                    // repozitionare
                    if(timer.seconds() > 0.05) {
                        state = STATE.TURN_3;
                        if(robot.glisieraMotorStanga.getTargetPosition() != 10) {
                            robot.glisieraMotorDreapta.setTargetPosition(-400);
                            robot.glisieraMotorStanga.setTargetPosition(-400);
                        }
                        setTargetTurnLeft(260);
                        runToPosition();
                        setPower(0.4);
                    }
                    break;
                case TURN_3:
                    if(!isBusy()) {
                        state = STATE.WAIT_5;
                        setPower(0);
                        resetEncoders();
                        timer.reset();
                    }
                    break;
                case WAIT_5:
                    // drive in fata
                    if(timer.seconds() > 0.05) {
                        state = STATE.FORWARD_4;
                        setTargetFoward(Navigation2.mmToTicks(570));
                        runToPosition();
                        setPower(0.4);
                    }
                    break;
                case FORWARD_4:
                    if(!isBusy()) {
                        state = STATE.WAIT_7;
                        setPower(0);
                        resetEncoders();
                        timer.reset();
                    }
                    break;
                case WAIT_7:
                    //turn 90 spre stack
                    if(timer.seconds() > 0.05) {
                        state = STATE.TURN_7;
                        setTargetTurnLeft(613);
                        runToPosition();
                        setPower(0.4);
                    }
                    break;
                case TURN_7:
                    if(!isBusy()) {
                        state = STATE.WAIT_8;
                        setPower(0);
                        resetEncoders();
                        timer.reset();
                    }
                    break;
                case WAIT_8:
                    // drive spre stack
                    if(timer.seconds() > 0.05) {
                        state = STATE.FORWARD_IN_STACK;
                        setTargetFoward(Navigation2.mmToTicks(735));
                        runToPosition();
                        setPower(0.3);
                    }
                    break;
                case FORWARD_IN_STACK:
                    // prinde si ridica conu
                    if(!isBusy()) {
                        state = STATE.WAIT_9;


                        robot.servoGripper.setPosition(ControlLisa.gripperInchis);
                        Thread.sleep(500);
                        robot.glisieraMotorStanga.setTargetPosition(-450);
                        robot.glisieraMotorDreapta.setTargetPosition(-450);
                        setPower(0);
                        resetEncoders();
                        timer.reset();
                    }
                    break;
                case WAIT_9:
                    // merge in spate putin ca sa puna con pe low cu fata
                    // seteaza pendulu la pozitia intermediar
                    // coboara liftu la poz low
                    if(timer.seconds() > 0.05) {
                        state = STATE.PUTIN_BACK_DIN_STACK;
                        setTargetFoward(-Navigation2.mmToTicks(560));
                        runToPosition();
                        setPower(0.4);
                        robot.servo_pendul_right.setPosition(ControlLisa.pendulDreaptaIntermediar);
                        robot.servo_pendul_left.setPosition(ControlLisa.pendulStangaIntermediar);
                        Thread.sleep(1000);
                        robot.glisieraMotorStanga.setTargetPosition(-100);
                        robot.glisieraMotorDreapta.setTargetPosition(-100);
                    }
                    break;
                case PUTIN_BACK_DIN_STACK:
                    if(!isBusy()) {
                        state = STATE.WAIT_10;
                        setPower(0);
                        resetEncoders();
                        timer.reset();
                    }
                    break;
                case WAIT_10:
                    //turn spre low
                    if(timer.seconds() > 0.05) {
                        state = STATE.ALLIGN_LOW;
                        setTargetTurnLeft(435);
                        runToPosition();
                        setPower(0.3);
                    }
                    break;
                case ALLIGN_LOW:
                    if(!isBusy()){
                        state = STATE.WAIT_11;
                        setPower(0);
                        resetEncoders();
                        timer.reset();
                    }
                    break;

                    // -------------------------- LOW JUNCTION -------------------------------------

                case WAIT_11:
                    // drive in fata deasura low junctionului
                    if(timer.seconds() > 0.05) {
                        state = STATE.MAI_IN_FATA_LA_LOW;
                        setTargetFoward(Navigation2.mmToTicks(160));
                        runToPosition();
                        setPower(0.4);
                    }
                    break;
                case MAI_IN_FATA_LA_LOW:
                    // lasa conu pe low, sper :)
                    if(!isBusy()) {
                        state = STATE.WAIT_12;
                        robot.servoGripper.setPosition(ControlLisa.gripperDeschis);
                        setPower(0);
                        resetEncoders();
                        timer.reset();
                    }
                    break;
                case WAIT_12:
                    if(timer.seconds() > 0.05) {
                        state = STATE.DRIVE_BACK;
                        setTargetFoward(-Navigation2.mmToTicks(160));
                        runToPosition();
                        setPower(0.4);
                    }
                    break;
                case DRIVE_BACK:
                    if(!isBusy()){
                        state = STATE.WAIT_13;
                        setPower(0);
                        resetEncoders();
                        timer.reset();
                    }
                    break;
                case WAIT_13:
                    // turn de la low junction spre stack
                    if(timer.seconds() > 0.05) {
                        state = STATE.SPRE_STACK;
                        setTargetTurnRight(458);
                        runToPosition();
                        setPower(0.3);
                    }
                    break;
                case SPRE_STACK:
                    if(!isBusy()) {
                        state = STATE.WAIT_14;
                        setPower(0);
                        resetEncoders();
                        timer.reset();
                    }
                    break;
                case WAIT_14:
                    // drive la stack
                    if(timer.seconds() > 0.05) {
                        state = STATE.DRIVE_LA_STACK;

                        robot.servo_pendul_right.setPosition(ControlLisa.pendulDreaptaPozitieInitiala);
                        robot.servo_pendul_left.setPosition(ControlLisa.pendulStangaPozitieInitiala);

                        Thread.sleep(500);

                        setTargetFoward(Navigation2.mmToTicks(577));
                        runToPosition();
                        setPower(0.3);

                        Thread.sleep(500);

                        robot.glisieraMotorStanga.setTargetPosition(-390);
                        robot.glisieraMotorDreapta.setTargetPosition(-390);


                    }
                    break;
                case DRIVE_LA_STACK:

                    if(!isBusy()) {
                        state = STATE.WAIT_15;

                        Thread.sleep(1000);
                        robot.servoGripper.setPosition(ControlLisa.gripperInchis);

                        setPower(0);
                        resetEncoders();
                        timer.reset();
                    }
                    break;
                case WAIT_15:

                    if(timer.seconds() > 0.05) {
                        state = STATE.DRIVE_BACK2;



                        robot.glisieraMotorStanga.setTargetPosition(-570);
                        robot.glisieraMotorDreapta.setTargetPosition(-570);

                        Thread.sleep(1000);

                        setTargetFoward(-Navigation2.mmToTicks(530));
                        runToPosition();
                        setPower(0.4);
                        robot.servo_pendul_right.setPosition(ControlLisa.pendulDreaptaIntermediar);
                        robot.servo_pendul_left.setPosition(ControlLisa.pendulStangaIntermediar);
                        Thread.sleep(1000);
                        robot.glisieraMotorStanga.setTargetPosition(-120);
                        robot.glisieraMotorDreapta.setTargetPosition(-120);

                    }
                    break;

                case DRIVE_BACK2:
                    if(!isBusy()) {
                        state = STATE.WAIT_16;

                        setPower(0);
                        resetEncoders();
                        timer.reset();
                    }
                    break;
                case WAIT_16:
                    if(timer.seconds() > 0.05) {


                            state = STATE.TURN_LA_LOW;
                            setTargetTurnLeft(435);
                            runToPosition();
                            setPower(0.3);

                    }
                    break;
                case TURN_LA_LOW:
                    if(!isBusy()) {
                        state = STATE.WAIT_17;

                        setPower(0);
                        resetEncoders();
                        timer.reset();
                    }
                    break;

                case WAIT_17:
                    if(timer.seconds() > 0.05) {
                        state = STATE.ALLIGN_LOW2;
                        setTargetFoward(Navigation2.mmToTicks(165));
                        runToPosition();
                        setPower(0.3);
                    }
                    break;
                case ALLIGN_LOW2:
                    if(!isBusy()) {
                        state = STATE.WAIT_18;

                        setPower(0);
                        resetEncoders();
                        timer.reset();
                    }
                    break;
                case WAIT_18:
                    if(timer.seconds() > 0.05){
                        state = STATE.STOP;
                        robot.servoGripper.setPosition(ControlLisa.gripperDeschis);
                    }
                    break;
                case STOP:
                    setPower(0);
                    break;
            }
            telemetry.addData("state", state);
            telemetry.addData("first angle", robot.imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle);
            telemetry.addData("second angle", robot.imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle);
            telemetry.addData("third angle", robot.imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);
            telemetry.addData("Time", runTime.seconds());
            telemetry.update();
        }
    }
}
