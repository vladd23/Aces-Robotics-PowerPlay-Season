package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

//@Config
//@Autonomous(name="AutoDreapta", group="Pushbot")
public class AutonomDreapta extends LinearOpMode  implements OpModeAddition
{
    Hardware robot = new Hardware();

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    FtcDashboard dashboard;


    static final double FEET_PER_METER = 3.28084; // 100 de cm

    public static double power = 0.3;
    public static double wait = 3;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;


    // Tag ID 1,2,3 din familia 36h11 (adica alea din PDF)
    // Astea s cazurile noastre, caz 1- stanga, caz 2- centru, caz 3- dreapta
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null; //aici stocam ce vede camera, la inceput null

    @Override
    public void runOpMode()
    {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.init(hardwareMap);
        robot.navigation.setOpModeAddition(this);
        robot.navigation.setHardwareMap(hardwareMap);
//        robot.navigation.setTelemetry(this, robot.tensorDetectionClass);
        robot.navigation.resetEncoders();
        dashboard = FtcDashboard.getInstance();

        // initializam camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        FtcDashboard.getInstance().startCameraStream(camera, 0);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
                dashboard.startCameraStream(camera, 120);

            }

            @Override
            public void onError(int errorCode)
            {
            //aici bagam ceva daca are erori , eventual sa faca cazu default
            }
        });

        telemetry.setMsTransmissionInterval(50);


        // De aici incepe dupa ce dam init
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            // face un vector cu ultimele detectari pe intervale de milisecunde

            if(currentDetections.size() != 0)
            {
                //daca o detectat ceva atunci lungimea vectorului > 0
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    // verificam daca ce o gasit camera ii unu dintre cazurile noastre

                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        // daca da, variabila aia nulla o salvam ca fiind cazul vazut de camera
                        tagOfInterest = tag;

                        // am gasit un tag bun
                        tagFound = true;
                        break;
                    }
                }
                // daca so gasit unu din cazurile noastre
                if(tagFound)
                {
                    // mesaj ca uraa so gasit
                    telemetry.addLine("Am gasit unul dintre cazuri!\n\n");
                    tagToTelemetry(tagOfInterest);
                }
                else
                // am gasit ceva, da nu ii unu dintre cazurile noastre
                {

                    telemetry.addLine("Am gasit ceva, da nu-i ce cautam noi :(");

                    // daca variabila aia ramane null, inseamna ca no vazut veci cod qr
                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("Nu vad niciun tag :/");
                    }
                    // daca variabila asta si o luat cazu, da amu nu mai vede, inseamna ca de cand so dat init
                    // o vazut un caz bun, da dupa so mutat
                    else
                    {
                        telemetry.addLine("\nSi totusi l-am vazut ultima data la: ");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            // aici intra daca no vazut de curand vreun tag
            else
            {
                telemetry.addLine("Nu vad niciun tag :/");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("Nu l-am vazut boss deloc de cand ai dat init");
                }
                else
                {
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
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

      /* Sa vedem ce caz ii */
        waitForStart();

    // cod pus con pe low, se termina langa perete
        robot.navigation.pusConLowDreapta(power);
        dashboard.stopCameraStream();
        if( tagOfInterest.id == LEFT){
            robot.navigation.SlideOnTime(5.5, -power);
            robot.navigation.DriveOnTime(3, 0.3); //doamne ajuta


        }else if(tagOfInterest == null || tagOfInterest.id == MIDDLE){
            // caz 2
            robot.navigation.SlideOnTime(1.5, -power);
            robot.navigation.DriveOnTime(3, 0.3); //doamne ajuta
        }

        else if(tagOfInterest.id == RIGHT){
            robot.navigation.SlideOnTime(1.5, power);
            robot.navigation.DriveOnTime(3, 0.3); //doamne ajuta

        }


    }
    // functie de adaug
    @SuppressLint("DefaultLocale")
    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nAm gasit cazu=%d", detection.id));
    }

    @Override
    public boolean isOpModeIsActive() {
        return opModeIsActive();
    }
}