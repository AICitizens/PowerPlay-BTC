package org.firstinspires.ftc.teamcode.drive.robo9u.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.drive.robo9u.ModdedHardware.ThreadedIMU;
import org.firstinspires.ftc.teamcode.drive.robo9u.Modules.Claw;
import org.firstinspires.ftc.teamcode.drive.robo9u.Modules.ConeAligner;
import org.firstinspires.ftc.teamcode.drive.robo9u.Modules.Lift;
import org.firstinspires.ftc.teamcode.drive.robo9u.Modules.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.drive.robo9u.Modules.Mechanisms;
import org.firstinspires.ftc.teamcode.drive.robo9u.Modules.SingleBar;

@TeleOp(name="Rami Field-Centric")
public class RamiFieldCentric extends LinearOpMode {
    private MecanumDrivetrain drive;
    private ThreadedIMU threadedIMU;
    private Mechanisms mecanisme;
    private ElapsedTime runtime;
    private GamepadEx gamepad;
    ElapsedTime clawTimer;

    public void initialize()
    {
        drive = new MecanumDrivetrain(hardwareMap, false);
        threadedIMU = new ThreadedIMU(hardwareMap);
        threadedIMU.startImuThread(this);
        mecanisme = new Mechanisms(hardwareMap);
        runtime = new ElapsedTime();

        mecanisme.lift.singleBar.setSingleBarState(SingleBar.SingleBarState.Front);
        mecanisme.lift.claw.Open();

        PhotonCore.EXPANSION_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.experimental.setMaximumParallelCommands(8);
        PhotonCore.enable();
        PhotonCore.EXPANSION_HUB.clearBulkCache();

        gamepad = new GamepadEx(gamepad1);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    public void updateClaw()
    {
        if(gamepad.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            if (mecanisme.lift.claw.safeServo.getPosition() == Claw.closedPosition) {
                mecanisme.lift.claw.Open();
            } else {
                mecanisme.lift.claw.Close();
            }
        }
        if(gamepad.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            if (mecanisme.coneAligner.servo.getPosition() == ConeAligner.up) {
                mecanisme.coneAligner.Down();
            } else {
                mecanisme.coneAligner.Up();
            }
        }
        if(gamepad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN) || gamepad.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
            mecanisme.lift.setLiftState(Lift.LiftState.Defence);
        }
        if(gamepad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN) || gamepad.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
            if(Lift.dropBy == 0)
                mecanisme.lift.dropByCM(5);
            else
                mecanisme.lift.dropByCM(0);
        }
    }

    public void updateLift() {
        mecanisme.lift.setPower(gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
        if (gamepad.wasJustPressed(GamepadKeys.Button.A)){ // auto control
            mecanisme.lift.setLiftState(Lift.LiftState.High);
        }else if(gamepad.wasJustPressed(GamepadKeys.Button.Y)){
            mecanisme.lift.setLiftState(Lift.LiftState.Mid);
        }else if(gamepad.wasJustPressed(GamepadKeys.Button.B)){
            mecanisme.lift.setLiftState(Lift.LiftState.Low);
        }else if(gamepad.wasJustPressed(GamepadKeys.Button.X)){
            mecanisme.lift.setLiftState(Lift.LiftState.Ground);
            mecanisme.lift.claw.Open();
        }
        if(gamepad.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON)) {
            mecanisme.lift.nextStack();
        }
    }
    public void updatetelemetry(){
        telemetry.addLine("Running at " + 1e9/runtime.nanoseconds() + "hz");
        telemetry.addLine("Current " + (PhotonCore.CONTROL_HUB.getCurrent(CurrentUnit.AMPS) + PhotonCore.EXPANSION_HUB.getCurrent(CurrentUnit.AMPS))
                + " amps");
        telemetry.addData("lift amperage", mecanisme.lift.lift.left.getCurrent(CurrentUnit.AMPS)+mecanisme.lift.lift.right.getCurrent(CurrentUnit.AMPS));
        runtime.reset();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        ElapsedTime ept = new ElapsedTime(0);
        ept.reset();
        while(!isStopRequested() && opModeIsActive())
        {
            PhotonCore.EXPANSION_HUB.clearBulkCache();
            gamepad.readButtons();
            if(gamepad.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON)) threadedIMU.resetYaw();
            drive.driveFieldCentric(gamepad.getLeftY(), gamepad.getLeftX(), gamepad.getRightX(), threadedIMU.getYaw());
            telemetry.addData("yaw", threadedIMU.getYaw());
            updateClaw();
            updateLift();
            mecanisme.update();
            updatetelemetry();
            telemetry.update();
        }
    }
}