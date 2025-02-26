package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
public class LocalizationTest extends LinearOpMode {
    private double x;
    private double y;
    private double x2;
    private double denominator;
    private double LF;
    private double RF;
    private double LR;
    private double RR;
    private final double drivepow = 1.0f;

    SampleMecanumDrive drive;

    public void setMotorPowers(double v, double v1, double v2, double v3) {
        drive.leftFront.setPower(v);
        drive.leftRear.setPower(v1);
        drive.rightRear.setPower(v2);
        drive.rightFront.setPower(v3);
    }

    public void runOpMode() throws InterruptedException {


        drive = new SampleMecanumDrive(hardwareMap);
        drive.imu.startImuThread(this);
        drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (!isStopRequested()) {
            x2 = gamepad1.right_stick_x;
            y = -gamepad1.left_stick_y * 1.1;
            x = gamepad1.left_stick_x;
            denominator = Math.max(1, Math.abs(x) + Math.abs(y) + Math.abs(x2));
            LF = (x + y + x2) / denominator * drivepow;
            RF = (-x + y - x2) / denominator * drivepow;
            LR = (-x + y + x2) / denominator * drivepow;
            RR = (x + y - x2) / denominator * drivepow;
            setMotorPowers(LF, LR, RR, RF);

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}
