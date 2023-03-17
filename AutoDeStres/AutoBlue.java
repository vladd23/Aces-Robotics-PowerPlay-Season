package org.firstinspires.ftc.teamcode.AutoDeStres;


import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.OpModeAddition;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

//@Autonomous()
public class AutoBlue extends LinearOpMode implements OpModeAddition{
    Hardware robot = new Hardware();
    public static final double      DriveValue = 2.43;
    public static final double      TurnValue = 2;
    public static final double      StrafeValue = 2;
    public static final double      PullValue = 125;
    public static final double      COUNTS_PER_MOTOR_REV = 1120 ;
    public static final double      DRIVE_GEAR_REDUCTION = 2.0 ;
    public static final double      WHEEL_DIAMETER_MM = 4.0 * 25.4;
    public static final double     COUNTS_PER_MM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_MM * 3.1415);
    public static final double      DRIVE_SPEED = 1;
    public static final double      TURN_SPEED = 0.5;
    public static final double      PULL_SPEED = 0.1;

    private ElapsedTime runtime;
    AprilTagDetection tagOfInterest = null;
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    double tagsize = 0.166;
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.navigation.setOpModeAddition(this);
        robot.navigation.setHardwareMap(hardwareMap);
        robot.navigation.setTelemetry(this);
        robot.navigation.imuInit();
        runtime = new ElapsedTime();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        FtcDashboard.getInstance().startCameraStream(camera, 0);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                //aici bagam ceva daca are erori , eventual sa faca cazu default
            }
        });

        telemetry.setMsTransmissionInterval(50);


        // De aici incepe dupa ce dam init
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            // face un vector cu ultimele detectari pe intervale de milisecunde

            if (currentDetections.size() != 0) {
                //daca o detectat ceva atunci lungimea vectorului > 0
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    // verificam daca ce o gasit camera ii unu dintre cazurile noastre

                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                        // daca da, variabila aia nulla o salvam ca fiind cazul vazut de camera
                        tagOfInterest = tag;

                        // am gasit un tag bun
                        tagFound = true;
                        break;
                    }
                }
                // daca so gasit unu din cazurile noastre
                if (tagFound) {
                    // mesaj ca uraa so gasit
                    telemetry.addLine("Am gasit unul dintre cazuri!\n\n");
                    tagToTelemetry(tagOfInterest);
                } else
                // am gasit ceva, da nu ii unu dintre cazurile noastre
                {

                    telemetry.addLine("Am gasit ceva, da nu-i ce cautam noi :(");

                    // daca variabila aia ramane null, inseamna ca no vazut veci cod qr
                    if (tagOfInterest == null) {
                        telemetry.addLine("Nu vad niciun tag :/");
                        tagOfInterest.id = MIDDLE;

                    }
                    // daca variabila asta si o luat cazu, da amu nu mai vede, inseamna ca de cand so dat init
                    // o vazut un caz bun, da dupa so mutat
                    else {
                        telemetry.addLine("\nSi totusi l-am vazut ultima data la: ");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            // aici intra daca no vazut de curand vreun tag
            else {
                telemetry.addLine("Nu vad niciun tag :/");

                if (tagOfInterest == null) {
                    telemetry.addLine("Nu l-am vazut boss deloc de cand ai dat init");
//                    tagOfInterest.id = MIDDLE;
                } else {
                    telemetry.addLine("\nSi totusi l-am vazut ultima data la:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if (tagOfInterest != null) {
            telemetry.addLine("Update:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
//            tagOfInterest.id = MIDDLE;
            telemetry.addLine("Inca n-am vazut nimic, fac cazu 2 :(");
            telemetry.update();
        }

        waitForStart();

        StrafeLeft(7, 0.3);
        if(tagOfInterest.id == LEFT) {
            DriveForward(50, 0.34, 0.34, 0.3, 0.3);
            StrafeLeft(60, 0.3);
        } else if(tagOfInterest.id == RIGHT) {
            DriveForward(50, 0.34, 0.34, 0.3, 0.3);
            StrafeRight(60, 0.3);
        } else {
            DriveForward(50, 0.34, 0.34, 0.3, 0.3);
        }

    }
    
    public void EncoderDrive (double speed_RF,double speed_RB,double speed_LF,double speed_LB, double distance, double timeoutS)
    {
        if (opModeIsActive()) {
            int newBackLeftTarget;
            int newBackRightTarget;
            int newFrontLeftTarget;
            int newFrontRightTarget;
            // Ensure that the opmode is still active
            if (opModeIsActive()) {

                // Determine new target position, and pass to motor controller
                newBackLeftTarget = robot.leftMotorBack.getCurrentPosition() + (int) (distance * COUNTS_PER_MM * DriveValue);
                newBackRightTarget = robot.rightMotorBack.getCurrentPosition() + (int) (distance * COUNTS_PER_MM * DriveValue);
                newFrontLeftTarget = robot.leftMotorFront.getCurrentPosition() + (int) (distance * COUNTS_PER_MM * DriveValue);
                newFrontRightTarget = robot.rightMotorFront.getCurrentPosition() + (int) (distance * COUNTS_PER_MM * DriveValue);

                robot.leftMotorBack.setTargetPosition(newBackLeftTarget);
                robot.rightMotorBack.setTargetPosition(newBackRightTarget);
                robot.leftMotorFront.setTargetPosition(newFrontLeftTarget);
                robot.rightMotorFront.setTargetPosition(newFrontRightTarget);

                // Turn On RUN_TO_POSITION
                robot.leftMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rightMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.leftMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rightMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // reset the timeout time and start motion.
                runtime.reset();
                robot.leftMotorBack.setPower(speed_LB);
                robot.rightMotorBack.setPower(speed_RB);
                robot.leftMotorFront.setPower(speed_LF);
                robot.rightMotorFront.setPower(speed_RF);

                // keep looping while we are still active, and there is time left, and both motors are running.
                // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
                // its target position, the motion will stop.  This is "safer" in the event that the drive will
                // always end the motion as soon as possible.
                // However, if you require that BOTH motors have finished their moves before the drive continues
                // onto the next step, use (isBusy() || isBusy()) in the loop test.
                while (opModeIsActive() &&
                        (runtime.seconds() < timeoutS) &&
                        (
                                robot.leftMotorBack.isBusy() && robot.rightMotorBack.isBusy() &&
                                        robot.leftMotorFront.isBusy() && robot.rightMotorFront.isBusy()
                        )) {

                    // Display it for the driver.
                    telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d"
                            , newFrontLeftTarget, newFrontRightTarget
                            , newBackLeftTarget, newBackRightTarget
                    );
                    telemetry.addData("Path2", "Running at %7d :%7d :%7d :%7d",
                            robot.leftMotorFront.getCurrentPosition(),
                            robot.rightMotorFront.getCurrentPosition()
                            ,
                            robot.leftMotorBack.getCurrentPosition(),
                            robot.rightMotorBack.getCurrentPosition()
                    );
                    telemetry.addData("RF", String.valueOf(robot.rightMotorFront.getPower()));
                    telemetry.addData("RB", String.valueOf(robot.rightMotorBack.getPower()));
                    telemetry.addData("LF", String.valueOf(robot.leftMotorFront.getPower()));
                    telemetry.addData("LB", String.valueOf(robot.leftMotorBack.getPower()));
                    telemetry.update();
                }

                // Stop all motion;


                robot.leftMotorBack.setPower(0);
                robot.rightMotorBack.setPower(0);
                robot.leftMotorFront.setPower(0);
                robot.rightMotorFront.setPower(0);


                /* COMMENT THESE FOR SPEED */

                // Turn off RUN_TO_POSITION
                robot.leftMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.leftMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                //sleep(250);   // optional pause after each move
            }
        }
    }
    public void EncoderStrafe(double speed, double distance, double timeoutS) {
        int newBackLeftTarget;
        int newBackRightTarget;
        int newFrontLeftTarget;
        int newFrontRightTarget;
        if (opModeIsActive()) {

            newBackLeftTarget = robot.leftMotorBack.getCurrentPosition() + (int)(distance * COUNTS_PER_MM * StrafeValue);
            newBackRightTarget = robot.rightMotorBack.getCurrentPosition() + (int)(-distance * COUNTS_PER_MM * StrafeValue);
            newFrontLeftTarget = robot.leftMotorFront.getCurrentPosition() + (int)(-distance * COUNTS_PER_MM * StrafeValue);
            newFrontRightTarget = robot.rightMotorFront.getCurrentPosition() + (int)(distance * COUNTS_PER_MM * StrafeValue);

            robot.leftMotorBack.setTargetPosition(newBackLeftTarget);
            robot.rightMotorBack.setTargetPosition(newBackRightTarget);
            robot.leftMotorFront.setTargetPosition(newFrontLeftTarget);
            robot.rightMotorFront.setTargetPosition(newFrontRightTarget);

            robot.leftMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            robot.leftMotorBack.setPower(-speed+0.04);
            robot.rightMotorBack.setPower(-speed-0.04);
            robot.leftMotorFront.setPower(speed-0.4);
            robot.rightMotorFront.setPower(speed+0.04);

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (
                            robot.leftMotorBack.isBusy() && robot.rightMotorBack.isBusy() &&
                                    robot.leftMotorFront.isBusy() && robot.rightMotorFront.isBusy()
                    ))
            {

                telemetry.addData("Path1",  "Running to %7d :%7d :%7d :%7d"
                        , newFrontLeftTarget, newFrontRightTarget
                        , newBackLeftTarget, newBackRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d",
                        robot.leftMotorFront.getCurrentPosition(),
                        robot.rightMotorFront.getCurrentPosition()
                        ,
                        robot.leftMotorBack.getCurrentPosition(),
                        robot.rightMotorBack.getCurrentPosition());

                telemetry.update();
            }

            robot.leftMotorBack.setPower(0);
            robot.rightMotorBack.setPower(0);
            robot.leftMotorFront.setPower(0);
            robot.rightMotorFront.setPower(0);

            robot.leftMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }

    public void DriveBackward (double distance, double speed_RF,double speed_RB,double speed_LF,double speed_LB)
    {
        EncoderDrive(speed_RF,speed_RB,speed_LF,speed_LB, distance,15);
        //sleep(100);
    }

    public void DriveForward (double distance, double speed_RF,double speed_RB,double speed_LF,double speed_LB)
    {
        EncoderDrive(speed_RF,speed_RB,speed_LF,speed_LB, -distance,15);
        //sleep(100);
    }

    public void StrafeRight(double distance, double speed)
    {
        EncoderStrafe(speed, distance, 15);
        sleep(100);
    }

    public void StrafeLeft (double distance, double speed)
    {
        EncoderStrafe(-speed, -distance, 15);
        sleep(100);
    }

    public void EncoderTurn(double speed, double distance, double timeoutS) {
        int newBackLeftTarget;
        int newBackRightTarget;
        int newFrontLeftTarget;
        int newFrontRightTarget;
        if (opModeIsActive()) {

            newBackLeftTarget = robot.leftMotorBack.getCurrentPosition() + (int)(-distance * COUNTS_PER_MM * TurnValue);
            newBackRightTarget = robot.rightMotorBack.getCurrentPosition() + (int)(distance * COUNTS_PER_MM * TurnValue);
            newFrontLeftTarget = robot.leftMotorFront.getCurrentPosition() + (int)(-distance * COUNTS_PER_MM * TurnValue);
            newFrontRightTarget = robot.rightMotorFront.getCurrentPosition() + (int)(distance * COUNTS_PER_MM * TurnValue);

            robot.leftMotorBack.setTargetPosition(newBackLeftTarget);
            robot.rightMotorBack.setTargetPosition(newBackRightTarget);
            robot.leftMotorFront.setTargetPosition(newFrontLeftTarget);
            robot.rightMotorFront.setTargetPosition(newFrontRightTarget);

            robot.leftMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            robot.leftMotorBack.setPower(speed);
            robot.rightMotorBack.setPower(speed);
            robot.leftMotorFront.setPower(speed);
            robot.rightMotorFront.setPower(speed);

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (
                            robot.leftMotorBack.isBusy() && robot.rightMotorBack.isBusy() &&
                                    robot.leftMotorFront.isBusy() && robot.rightMotorFront.isBusy()
                    ))
            {
                telemetry.addData("Path1",  "Running to %7d :%7d :%7d :%7d"
                        , newFrontLeftTarget, newFrontRightTarget
                        , newBackLeftTarget, newBackRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d",
                        robot.leftMotorFront.getCurrentPosition(),
                        robot.rightMotorFront.getCurrentPosition()
                        ,
                        robot.leftMotorBack.getCurrentPosition(),
                        robot.rightMotorBack.getCurrentPosition());
                telemetry.update();
            }

            //robot.leftMotorBack.setPower(0);
            //robot.rightMotorBack.setPower(0);
            //robot.leftMotorFront.setPower(0);
            //robot.rightMotorFront.setPower(0);

            robot.leftMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }

    public void RotateRight(double angle)
    {
        EncoderTurn(TURN_SPEED, angle, 15);
    }

    public void RotateLeft(double angle)
    {
        EncoderTurn(-TURN_SPEED, -angle, 15);
    }

    public void StopAllMotion() {
        robot.leftMotorFront.setPower(0);
        robot.rightMotorFront.setPower(0);
        robot.leftMotorBack.setPower(0);
        robot.rightMotorBack.setPower(0);

    }

    public void runUsingEncoders(double p, int ticks, double timeout) {

        runtime.reset();

        robot.leftMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.leftMotorFront.setTargetPosition(-ticks);
        robot.leftMotorBack.setTargetPosition(-ticks);
        robot.rightMotorFront.setTargetPosition(-ticks);
        robot.rightMotorBack.setTargetPosition(-ticks);

        robot.leftMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (robot.rightMotorFront.isBusy() && robot.rightMotorBack.isBusy() && robot.leftMotorFront.isBusy() && robot.leftMotorBack.isBusy()
                && runtime.seconds() < timeout){
            robot.leftMotorFront.setPower(p);
            robot.leftMotorBack.setPower(p);
            robot.rightMotorFront.setPower(p);
            robot.rightMotorBack.setPower(p);
        }

        StopAllMotion();
    }


    public int convertire(double cm){
        int ticks = 563;
        int cmperrotatie=30;
        int x = (int)(cm*ticks)/cmperrotatie;
        return x;
    }

    @Override
    public boolean isOpModeIsActive(){return opModeIsActive();}

    @SuppressLint("DefaultLocale")
    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nAm gasit cazu=%d", detection.id));
    }
}
