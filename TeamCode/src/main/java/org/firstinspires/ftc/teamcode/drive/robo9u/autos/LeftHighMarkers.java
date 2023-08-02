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
public class LeftHighMarkers extends LinearOpMode {
    Mechanisms mecanisme;
    SampleMecanumDrive drive;
    Detection detection;
    TrajectorySequence fullTrajectory;
    Trajectory[] gotoPark = new Trajectory[3];
    public static double averageXError = -1.9, averageYError = -0.9;
    public static double junctionX = -28.61, junctionY = -7.64, junctionHeading = 55.54;
    public static double stackX = -60.35, stackY = -11.66, stackHeading = 180.00;
    int conesPlaced = 0;
    private boolean parking = false;

    private ElapsedTime runtime;

    public void initialize()
    {
        drive = new SampleMecanumDrive(hardwareMap);
        drive.imu.startImuThread(this);
        mecanisme = new Mechanisms(hardwareMap);
        detection = new Detection(hardwareMap, "Webcam 0");
        runtime = new ElapsedTime();
        detection.setSleeveDetectionMode();

        Vector2d  junctionVector = new Vector2d(junctionX, junctionY),
                stackVector = new Vector2d(stackX, stackY);
        Pose2d averageErrorPose = new Pose2d(averageXError, averageYError, 0);

        fullTrajectory = drive.trajectorySequenceBuilder(new Pose2d(-35.02, -64.43, Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(0.25, ()->mecanisme.lift.setLiftState(Lift.LiftState.Mid))
                .splineTo(junctionVector, Math.toRadians(junctionHeading))
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{ mecanisme.lift.claw.Open();drive.setPoseEstimate(drive.getPoseEstimate().minus(averageErrorPose));})
                .UNSTABLE_addTemporalMarkerOffset(1, ()->mecanisme.lift.nextStack())
                .setReversed(true)
                .splineTo(stackVector, Math.toRadians(stackHeading))
                .UNSTABLE_addTemporalMarkerOffset(-0.05, ()->mecanisme.lift.claw.Close())
                .UNSTABLE_addTemporalMarkerOffset(0.1, ()->mecanisme.lift.setLiftState(Lift.LiftState.Mid))
                .setReversed(false)
                .splineTo(junctionVector, Math.toRadians(junctionHeading))
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{ mecanisme.lift.claw.Open();drive.setPoseEstimate(drive.getPoseEstimate().minus(averageErrorPose));})
                .UNSTABLE_addTemporalMarkerOffset(1, ()->mecanisme.lift.nextStack())
                .setReversed(true)
                .splineTo(stackVector, Math.toRadians(stackHeading))
                .UNSTABLE_addTemporalMarkerOffset(-0.05, ()->mecanisme.lift.claw.Close())
                .UNSTABLE_addTemporalMarkerOffset(0.1, ()->mecanisme.lift.setLiftState(Lift.LiftState.Mid))
                .setReversed(false)
                .splineTo(junctionVector, Math.toRadians(junctionHeading))
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{ mecanisme.lift.claw.Open();drive.setPoseEstimate(drive.getPoseEstimate().minus(averageErrorPose));})
                .UNSTABLE_addTemporalMarkerOffset(1, ()->mecanisme.lift.nextStack())
                .setReversed(true)
                .splineTo(stackVector, Math.toRadians(stackHeading))
                .UNSTABLE_addTemporalMarkerOffset(-0.05, ()->mecanisme.lift.claw.Close())
                .UNSTABLE_addTemporalMarkerOffset(0.1, ()->mecanisme.lift.setLiftState(Lift.LiftState.Mid))
                .setReversed(false)
                .splineTo(junctionVector, Math.toRadians(junctionHeading))
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{ mecanisme.lift.claw.Open();drive.setPoseEstimate(drive.getPoseEstimate().minus(averageErrorPose));})
                .UNSTABLE_addTemporalMarkerOffset(1, ()->mecanisme.lift.nextStack())
                .setReversed(true)
                .splineTo(stackVector, Math.toRadians(stackHeading))
                .UNSTABLE_addTemporalMarkerOffset(-0.05, ()->mecanisme.lift.claw.Close())
                .UNSTABLE_addTemporalMarkerOffset(0.1, ()->mecanisme.lift.setLiftState(Lift.LiftState.Mid))
                .setReversed(false)
                .splineTo(junctionVector, Math.toRadians(junctionHeading))
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{ mecanisme.lift.claw.Open();drive.setPoseEstimate(drive.getPoseEstimate().minus(averageErrorPose));})
                .UNSTABLE_addTemporalMarkerOffset(1, ()->mecanisme.lift.nextStack())
                .setReversed(true)
                .splineTo(stackVector, Math.toRadians(stackHeading))
                .UNSTABLE_addTemporalMarkerOffset(-0.05, ()->mecanisme.lift.claw.Close())
                .UNSTABLE_addTemporalMarkerOffset(0.1, ()->mecanisme.lift.setLiftState(Lift.LiftState.Mid))
                .setReversed(false)
                .splineTo(junctionVector, Math.toRadians(junctionHeading))
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{ mecanisme.lift.claw.Open();drive.setPoseEstimate(drive.getPoseEstimate().minus(averageErrorPose));})
                .UNSTABLE_addTemporalMarkerOffset(1, ()->mecanisme.lift.nextStack())
                .build();

        gotoPark[0] = drive.trajectoryBuilder(fullTrajectory.end()).lineToLinearHeading(new Pose2d(52.5, 28, Math.toRadians(-180))).build();
        gotoPark[1] = drive.trajectoryBuilder(fullTrajectory.end()).lineToLinearHeading(new Pose2d(52.5,  -2, Math.toRadians(-180))).build();
        gotoPark[2] = drive.trajectoryBuilder(fullTrajectory.end()).lineToLinearHeading(new Pose2d(52.5, -24, Math.toRadians(-180))).build();

        PhotonCore.EXPANSION_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.experimental.setMaximumParallelCommands(8);
        PhotonCore.enable();
        PhotonCore.EXPANSION_HUB.clearBulkCache();

        mecanisme.lift.claw.Close();
        mecanisme.lift.singleBar.Up();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
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