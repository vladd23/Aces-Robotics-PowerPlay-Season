package org.firstinspires.ftc.teamcode;

import android.graphics.Path;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.io.IOException;
import java.util.ArrayList;

//@Config
//@Autonomous(name="AutonomousTest", group="Pushbot")


public class AutonomousTest extends LinearOpMode implements OpModeAddition {

    Hardware robot = new Hardware();
    public static double time = 1.4;
    public static double speed = 0.5;
    public static double angle = 35;


    @Override
    public boolean isOpModeIsActive(){
        return opModeIsActive();
    }

    @Override
    public void runOpMode() {


        robot.init(hardwareMap);
        robot.navigation.setOpModeAddition(this);
        robot.navigation.setHardwareMap(hardwareMap);
        robot.navigation.setTelemetry(this);
        robot.navigation.imuInit();

        waitForStart();


// -------------------- de testat --------------------------

//        robot.navigation.pendulSpate();
//        new Thread(() -> {
//                try {
//                    Thread.sleep(100);
//                    robot.navigation.DriveOnTime(time, -speed);
//
//                } catch (InterruptedException e) {
//                    e.printStackTrace();
//                }
//            }).start();


        // UNGHIURILE MERG CU - 10 GRADE LA VITEZA 0.3
        //AMINNNN
          robot.navigation.driveOnlyBack(-30, -0.5);
//          robot.navigation.DriveOnTime(time, -speed);
//        robot.navigation.waitUntil(2);
//        robot.navigation.Turn(angle, 0.3);
//        robot.navigation.DriveOnTime(.3,-0.3);
//        robot.navigation.waitUntil(2);
//        robot.navigation.pendulFata();
//        robot.navigation.waitUntil(2);


//        robot.navigation.ResetAngle();
       // telemetry.update();


    }


}