package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

//@TeleOp(name = "Test ultrasonic")
public class TestUltrasonic extends OpMode {
    ModernRoboticsI2cRangeSensor sensor;
    FtcDashboard dashboard;
    Telemetry telemetry;

    @Override
    public void init() {
        sensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor");
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());
    }

    @Override
    public void loop() {
        telemetry.addData("Ultrasonic", sensor.cmUltrasonic());
        telemetry.addData("Optical", sensor.cmOptical());
    }
}