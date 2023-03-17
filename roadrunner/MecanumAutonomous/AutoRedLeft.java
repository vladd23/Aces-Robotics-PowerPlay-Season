package org.firstinspires.ftc.teamcode.roadrunner.MecanumAutonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.drive.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Coordonate red left")
public class AutoRedLeft extends LinearOpMode {
    MecanumDrive drive;
    FtcDashboard dashboard;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDrive(hardwareMap);
        dashboard = FtcDashboard.getInstance();
        drive.setPoseEstimate(new Pose2d(-35.8, -63, Math.toRadians(90)));
        waitForStart();
        TrajectorySequence parcareMid = drive.trajectorySequenceBuilder(new Pose2d(-35.8, -63, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-35.8, -35.4, Math.toRadians(90)))
                .turn(Math.toRadians(90))
                .build();
        drive.followTrajectorySequence(parcareMid);
    }
}
