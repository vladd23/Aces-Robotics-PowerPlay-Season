package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//@Autonomous
public class TestAuto extends LinearOpMode {
    Navigation2 nav;

    @Override
    public void runOpMode() throws InterruptedException {
        nav = new Navigation2(hardwareMap, this);

        waitForStart();

        nav.foward(500);
    }
}
