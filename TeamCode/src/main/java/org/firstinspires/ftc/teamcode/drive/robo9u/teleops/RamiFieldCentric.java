package org.firstinspires.ftc.teamcode.drive.robo9u.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.ModdedHardware.ThreadedIMU;
import org.firstinspires.ftc.teamcode.drive.robo9u.Modules.Lift;
import org.firstinspires.ftc.teamcode.drive.robo9u.Modules.Mechanisms;

@TeleOp(name="Rami Field-Centric")
public class RamiFieldCentric extends LinearOpMode {
    private MecanumDrive drive;
    private ThreadedIMU threadedIMU;
    private Mechanisms mecanisme;
    private ElapsedTime runtime;
    private final GamepadEx gamepadEx = new GamepadEx(gamepad1);

    public void initialize()
    {
        drive = new MecanumDrive(new Motor(hardwareMap, "rightRear"), new Motor(hardwareMap, "leftRear"), new Motor(hardwareMap, "rightFront"), new Motor(hardwareMap, "leftFront"));
        threadedIMU = new ThreadedIMU(hardwareMap);
        threadedIMU.startImuThread(this);
        mecanisme = new Mechanisms(hardwareMap);
        runtime = new ElapsedTime();

        mecanisme.lift.singleBar.Front();
        mecanisme.claw.Open();

        PhotonCore.EXPANSION_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.experimental.setMaximumParallelCommands(8);
        PhotonCore.enable();
        PhotonCore.EXPANSION_HUB.clearBulkCache();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    public void updateClaw()
    {
        if(gamepadEx.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)){
            mecanisme.claw.Close();
        }else if(gamepadEx.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)){
            mecanisme.claw.Open();
        }
        if(gamepadEx.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)){
            mecanisme.lift.singleBar.Front();
        }else if(gamepadEx.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
            mecanisme.lift.singleBar.Back();
        }
        if(gamepadEx.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)){
            mecanisme.coneAligner.Up();
        }else if(gamepadEx.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)){
            mecanisme.coneAligner.Down();
        }
        if(gamepadEx.wasJustPressed(GamepadKeys.Button.BACK))
            mecanisme.lift.setLiftState(Lift.LiftState.Defence);
    }

    public void updateLift() {
        mecanisme.lift.setPower(gamepadEx.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - gamepadEx.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
        if (gamepadEx.wasJustPressed(GamepadKeys.Button.A)){ // auto control
            mecanisme.lift.setLiftState(Lift.LiftState.High);
        }else if(gamepadEx.wasJustPressed(GamepadKeys.Button.Y)){
            mecanisme.lift.setLiftState(Lift.LiftState.Mid);
        }else if(gamepadEx.wasJustPressed(GamepadKeys.Button.B)){
            mecanisme.lift.setLiftState(Lift.LiftState.Low);
        }else if(gamepadEx.wasJustPressed(GamepadKeys.Button.X)){
            mecanisme.lift.setLiftState(Lift.LiftState.Ground);
            mecanisme.claw.Open();
        }
        if(gamepadEx.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON)) {
            mecanisme.lift.nextStack();
            mecanisme.claw.Open();
        }

    }
    public void updatetelemetry(){
        telemetry.addLine("Running at " + 1e9/runtime.nanoseconds() + "hz");
        telemetry.addLine("Current" + (PhotonCore.CONTROL_HUB.getCurrent(CurrentUnit.MILLIAMPS) + PhotonCore.EXPANSION_HUB.getCurrent(CurrentUnit.MILLIAMPS))/1000 + "amps");
        runtime.reset();
        telemetry.update();
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
            gamepadEx.readButtons();
            if(gamepadEx.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON)) threadedIMU.resetYaw();
            drive.driveFieldCentric(gamepadEx.getLeftX(), gamepadEx.getLeftY(), gamepadEx.getRightX(), threadedIMU.getYaw());
            updateClaw();
            updateLift();
            mecanisme.update();
            updatetelemetry();
        }
    }
}