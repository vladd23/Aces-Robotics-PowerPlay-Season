package org.firstinspires.ftc.teamcode.roadrunner;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.teamcode.roadrunner.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.roadrunner.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.roadrunner.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.roadrunner.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.roadrunner.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.roadrunner.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.roadrunner.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.roadrunner.DriveConstants.encoderTicksToInches;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.acmerobotics.roadrunner.control.PIDCoefficients;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptTensorFlowObjectDetection;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Navigation;
import org.firstinspires.ftc.teamcode.roadrunner.drive.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.drive.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.roadrunner.drive.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.roadrunner.drive.util.LynxModuleUtil;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.kV;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Config
public class MecanumDrive extends com.acmerobotics.roadrunner.drive.MecanumDrive {
    public DcMotorEx leftMotorBack = null;
    public DcMotorEx leftMotorFront = null;
    public DcMotorEx rightMotorBack = null;
    public DcMotorEx rightMotorFront = null;
//    public DcMotorEx glisieraMotorDreapta = null;
//    public DcMotorEx glisieraMotorStanga = null;

//    public DcMotor glisieraMotor = null;
//    public Navigation navigation = null;

    public BNO055IMU imu;


    public ModernRoboticsI2cRangeSensor distance_spate = null;
    public Servo servoGripper = null;
    public Servo servo_pendul_right = null;
    public Servo servo_pendul_left = null;

    public CRServo servo2 = null;
    public List<DcMotorEx> motors;

    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(0, 0, 0);

    public static PIDCoefficients HEADING_PID = new PIDCoefficients(0, 0, 0);


    public static double LATERAL_MULTIPLIER = 1;

    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    private TrajectorySequenceRunner trajectorySequenceRunner;

    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);

    private TrajectoryFollower follower;
    private VoltageSensor batteryVoltageSensor;

    public MecanumDrive(HardwareMap hmap) {
        super(kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);

        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);

//        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);
//        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

//        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
//            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
//        }

        // TODO: adjust the names of the following hardware devices to match your configuration
        imu = hmap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        // TODO: If the hub containing the IMU you are using is mounted so that the "REV" logo does
        // not face up, remap the IMU axes so that the z-axis points upward (normal to the floor.)
        //
        //             | +Z axis
        //             |
        //             |
        //             |
        //      _______|_____________     +Y axis
        //     /       |_____________/|__________
        //    /   REV / EXPANSION   //
        //   /       / HUB         //
        //  /_______/_____________//
        // |_______/_____________|/
        //        /
        //       / +X axis
        //
        // This diagram is derived from the axes in section 3.4 https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf
        // and the placement of the dot/orientation from https://docs.revrobotics.com/rev-control-system/control-system-overview/dimensions#imu-location
        //
        // For example, if +Y in this diagram faces downwards, you would use AxisDirection.NEG_Y.
        // BNO055IMUUtil.remapZAxis(imu, AxisDirection.NEG_Y);

        leftMotorFront = hmap.get(DcMotorEx.class, "left_motor_front");
        leftMotorBack = hmap.get(DcMotorEx.class, "left_motor_back");
        rightMotorBack = hmap.get(DcMotorEx.class, "right_motor_back");
        rightMotorFront = hmap.get(DcMotorEx.class, "right_motor_front");

//        glisieraMotorDreapta = hmap.get(DcMotorEx.class, "glisiera_motor_right");
//        glisieraMotorStanga = hmap.get(DcMotorEx.class, "glisiera_motor_left");

//        servoGripper = hmap.get(Servo.class, "servo_gripper");

//        servo_pendul_right = hmap.get(Servo.class, "servo_pendul_right");
//        servo_pendul_left = hmap.get(Servo.class, "servo_pendul_left");

        leftMotorBack.setDirection(DcMotor.Direction.REVERSE);
        leftMotorFront.setDirection(DcMotor.Direction.REVERSE);

//        glisieraMotorStanga.setDirection(DcMotor.Direction.REVERSE);
//
//        glisieraMotorStanga.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        glisieraMotorDreapta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        glisieraMotorStanga.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        glisieraMotorDreapta.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        glisieraMotorDreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        glisieraMotorStanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motors = Arrays.asList(leftMotorBack, leftMotorFront, rightMotorBack, rightMotorFront);
        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1);
            motor.setMotorType(motorConfigurationType);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        if (RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        trajectorySequenceRunner = new TrajectorySequenceRunner(follower, HEADING_PID);

//        IMU.Parameters parameters = new IMU.Parameters(
//                new RevHubOrientationOnRobot(
//                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
//                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
//                )
//        );
//        imu.initialize(parameters);

//        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

//        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
//            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
//        }
    }

    /**
     * Trajectory builder trajectory builder.
     *
     * @param startPose the start pose
     * @return the trajectory builder
     */
    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    /**
     * Trajectory builder trajectory builder.
     *
     * @param startPose the start pose
     * @param reversed  the reversed
     * @return the trajectory builder
     */
    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    /**
     * Trajectory builder trajectory builder.
     *
     * @param startPose    the start pose
     * @param startHeading the start heading
     * @return the trajectory builder
     */
    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    /**
     * Trajectory sequence builder trajectory sequence builder.
     *
     * @param startPose the start pose
     * @return the trajectory sequence builder
     */
    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                VEL_CONSTRAINT, ACCEL_CONSTRAINT,
                MAX_ANG_VEL, MAX_ANG_ACCEL
        );
    }

    /**
     * Turn async.
     *
     * @param angle the angle
     */
    public void turnAsync(double angle) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(getPoseEstimate())
                        .turn(angle)
                        .build()
        );
    }

    /**
     * Turn.
     *
     * @param angle the angle
     */
    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    /**
     * Follow trajectory async.
     *
     * @param trajectory the trajectory
     */
    public void followTrajectoryAsync(Trajectory trajectory) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(trajectory.start())
                        .addTrajectory(trajectory)
                        .build()
        );
    }

    /**
     * Follow trajectory.
     *
     * @param trajectory the trajectory
     */
    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    /**
     * Follow trajectory sequence async.
     *
     * @param trajectorySequence the trajectory sequence
     */
    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
    }

    /**
     * Follow trajectory sequence.
     *
     * @param trajectorySequence the trajectory sequence
     */
    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        followTrajectorySequenceAsync(trajectorySequence);
        waitForIdle();
    }

    /**
     * Gets last error.
     *
     * @return the last error
     */
    public Pose2d getLastError() {
        return trajectorySequenceRunner.getLastPoseError();
    }

    /**
     * Update.
     */
    public void update() {
        updatePoseEstimate();
        DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
        if (signal != null) setDriveSignal(signal);
    }

    /**
     * Wait for idle.
     */
    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy())
            update();
    }

    /**
     * Is busy boolean.
     *
     * @return the boolean
     */
    public boolean isBusy() {
        return trajectorySequenceRunner.isBusy();
    }

    /**
     * Sets mode.
     *
     * @param runMode the run mode
     */
    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    /**
     * Sets zero power behavior.
     *
     * @param zeroPowerBehavior the zero power behavior
     */
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    /**
     * Sets pidf coefficients.
     *
     * @param runMode      the run mode
     * @param coefficients the coefficients
     */
    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f // * 12 // / batteryVoltageSensor.getVoltage()
        );

        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    /**
     * Sets weighted drive power.
     *
     * @param drivePower the drive power
     */
    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + VY_WEIGHT * Math.abs(drivePower.getY())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        setDrivePower(vel);
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelPositions.add(encoderTicksToInches(motor.getCurrentPosition()));
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelVelocities.add(encoderTicksToInches(motor.getVelocity()));
        }
        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftMotorFront.setPower(v);
        leftMotorBack.setPower(v1);
        rightMotorBack.setPower(v2);
        rightMotorFront.setPower(v3);
    }

    @Override
    public double getRawExternalHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

    @Override
    public Double getExternalHeadingVelocity() {
        return (double) imu.getAngularVelocity().zRotationRate;
    }

    /**
     * Gets velocity constraint.
     *
     * @param maxVel        the max vel
     * @param maxAngularVel the max angular vel
     * @param trackWidth    the track width
     * @return the velocity constraint
     */
    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    /**
     * Gets acceleration constraint.
     *
     * @param maxAccel the max accel
     * @return the acceleration constraint
     */
    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }
}