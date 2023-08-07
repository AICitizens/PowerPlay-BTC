package org.firstinspires.ftc.teamcode.drive.robo9u.autos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.robo9u.Modules.Detection;
import org.firstinspires.ftc.teamcode.drive.robo9u.Modules.Lift;
import org.firstinspires.ftc.teamcode.drive.robo9u.Modules.Mechanisms;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(group="Demo", preselectTeleOp = "localiz_rami_field_centric")
public class LeftMid extends LinearOpMode {
    Mechanisms mecanisme;
    SampleMecanumDrive drive;
    Detection detection;
    TrajectorySequence fullTrajectory;
    Trajectory[] gotoPark = new Trajectory[3];
    public static double averageXError = 0.75, averageYError = 0;
    public static double junctionX = -31, junctionY = -18.5, junctionHeading = -45;
    public static double stackX = -60, stackY = -11.5, stackHeading = 180.00;
    int conesPlaced = 0;
    private boolean parking = false;

    private ElapsedTime runtime;

    public void initialize()
    {
        drive = new SampleMecanumDrive(hardwareMap);
        drive.imu.startImuThread(this);
        mecanisme = new Mechanisms(hardwareMap);
        mecanisme.lift.lift.stopAndResetEncoders();
        detection = new Detection(hardwareMap, "Webcam 0");
        runtime = new ElapsedTime();
        detection.setSleeveDetectionMode();

        Vector2d  junctionVector = new Vector2d(junctionX, junctionY),
                stackVector = new Vector2d(stackX, stackY);
        Pose2d averageErrorPose = new Pose2d(averageXError, averageYError, 0);

        fullTrajectory = drive.trajectorySequenceBuilder(new Pose2d(-35.02, -64.43, Math.toRadians(-90)))
                .setReversed(true)
                .UNSTABLE_addTemporalMarkerOffset(0.25, ()->mecanisme.lift.setLiftState(Lift.LiftState.Mid))
                .splineTo(junctionVector, Math.toRadians(junctionHeading))
                .UNSTABLE_addTemporalMarkerOffset(0.2, ()->{mecanisme.lift.nextStack();drive.setPoseEstimate(drive.getPoseEstimate().minus(averageErrorPose));})
                .waitSeconds(0.5)
                .setReversed(false)
                .splineTo(stackVector, Math.toRadians(stackHeading))
                .UNSTABLE_addTemporalMarkerOffset(0.2, ()->mecanisme.lift.claw.Close())
                .UNSTABLE_addTemporalMarkerOffset(0.4, ()->mecanisme.lift.setLiftState(Lift.LiftState.Mid))
                .setReversed(true)
                .waitSeconds(0.5)
                .splineTo(junctionVector, Math.toRadians(junctionHeading))
                .UNSTABLE_addTemporalMarkerOffset(0.2, ()->{mecanisme.lift.nextStack();drive.setPoseEstimate(drive.getPoseEstimate().minus(averageErrorPose));})
                .waitSeconds(0.5)
                .setReversed(false)
                .splineTo(stackVector, Math.toRadians(stackHeading))
                .UNSTABLE_addTemporalMarkerOffset(0.2, ()->mecanisme.lift.claw.Close())
                .UNSTABLE_addTemporalMarkerOffset(0.4, ()->mecanisme.lift.setLiftState(Lift.LiftState.Mid))
                .setReversed(true)
                .waitSeconds(0.5)
                .splineTo(junctionVector, Math.toRadians(junctionHeading))
                .UNSTABLE_addTemporalMarkerOffset(0.2, ()->{mecanisme.lift.nextStack();drive.setPoseEstimate(drive.getPoseEstimate().minus(averageErrorPose));})
                .waitSeconds(0.5)
                .setReversed(false)
                .splineTo(stackVector, Math.toRadians(stackHeading))
                .UNSTABLE_addTemporalMarkerOffset(0.2, ()->mecanisme.lift.claw.Close())
                .UNSTABLE_addTemporalMarkerOffset(0.4, ()->mecanisme.lift.setLiftState(Lift.LiftState.Mid))
                .setReversed(true)
                .waitSeconds(0.5)
                .splineTo(junctionVector, Math.toRadians(junctionHeading))
                .UNSTABLE_addTemporalMarkerOffset(0.2, ()->{mecanisme.lift.nextStack();drive.setPoseEstimate(drive.getPoseEstimate().minus(averageErrorPose));})
                .waitSeconds(0.5)
                .setReversed(false)
                .splineTo(stackVector, Math.toRadians(stackHeading))
                .UNSTABLE_addTemporalMarkerOffset(0.2, ()->mecanisme.lift.claw.Close())
                .UNSTABLE_addTemporalMarkerOffset(0.4, ()->mecanisme.lift.setLiftState(Lift.LiftState.Mid))
                .setReversed(true)
                .waitSeconds(0.5)
                .splineTo(junctionVector, Math.toRadians(junctionHeading))
                .UNSTABLE_addTemporalMarkerOffset(0.2, ()->{mecanisme.lift.nextStack();drive.setPoseEstimate(drive.getPoseEstimate().minus(averageErrorPose));})
                .waitSeconds(0.5)
                .setReversed(false)
                .splineTo(stackVector, Math.toRadians(stackHeading))
                .UNSTABLE_addTemporalMarkerOffset(0.2, ()->mecanisme.lift.claw.Close())
                .UNSTABLE_addTemporalMarkerOffset(0.4, ()->mecanisme.lift.setLiftState(Lift.LiftState.Mid))
                .setReversed(true)
                .waitSeconds(0.5)
                .splineTo(junctionVector, Math.toRadians(junctionHeading))
                .UNSTABLE_addTemporalMarkerOffset(0.2, ()->{mecanisme.lift.setLiftState(Lift.LiftState.Ground);drive.setPoseEstimate(drive.getPoseEstimate().minus(averageErrorPose));})
                .build();
        drive.setPoseEstimate(fullTrajectory.start());
        gotoPark[0] = drive.trajectoryBuilder(fullTrajectory.end()).lineToLinearHeading(new Pose2d(52.5, 28, Math.toRadians(-180))).build();
        gotoPark[1] = drive.trajectoryBuilder(fullTrajectory.end()).lineToLinearHeading(new Pose2d(52.5,  -2, Math.toRadians(-180))).build();
        gotoPark[2] = drive.trajectoryBuilder(fullTrajectory.end()).lineToLinearHeading(new Pose2d(52.5, -24, Math.toRadians(-180))).build();

        PhotonCore.EXPANSION_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.experimental.setMaximumParallelCommands(8);
        PhotonCore.enable();
        PhotonCore.EXPANSION_HUB.clearBulkCache();

        mecanisme.lift.claw.Close();
        mecanisme.lift.setLiftState(Lift.LiftState.Defence);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        while(!isStopRequested() && opModeInInit()){
            PhotonCore.EXPANSION_HUB.clearBulkCache();
            mecanisme.update();
            telemetry.addData("detectie", detection.getParkingIndex());
            telemetry.update();
        }
        waitForStart();
        detection.stopCamera();
        runtime.reset();
        drive.followTrajectorySequenceAsync(fullTrajectory);
        while(!isStopRequested() && opModeIsActive()) {
            PhotonCore.EXPANSION_HUB.clearBulkCache();

            if(!drive.isBusy() && !parking) {
                drive.followTrajectoryAsync(gotoPark[detection.getParkingIndex()]);
                parking = true;
            }
            mecanisme.update();
            drive.update();
            telemetry.addLine(conesPlaced + " cones placed");
            telemetry.addLine("Running at " + 1e9/runtime.nanoseconds() + "hz");
            runtime.reset();
            telemetry.update();
        }
        mecanisme.lift.stopCurrentTrajectory();
        drive.breakFollowing();
        SampleMecanumDrive.lastAutonomousPosition = drive.getPoseEstimate();
    }
}