package org.firstinspires.ftc.teamcode.drive.robo9u.teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.robo9u.Modules.Mechanisms;

@TeleOp(name="localiz_rami")
public class localizrami extends LinearOpMode {


    SampleMecanumDrive drive;
    Mechanisms mecanisme;

    public void initialize()
    {
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        mecanisme = new Mechanisms(hardwareMap);
    }

    Boolean lastbutton = false;
    double drivepow = 0.8;
    public void updateDrivePowers()
    {
        double LF, RF, LR, RR;
        double forward, rotate, strafe, denominator;
        if(gamepad1.right_stick_button && !lastbutton){
            drivepow = (drivepow == 0.8)?0.6:0.8;
        }
        lastbutton = gamepad1.right_stick_button;
        rotate=gamepad1.right_stick_x;
        forward=-gamepad1.left_stick_y*1.1;
        strafe=gamepad1.left_stick_x;
        denominator = Math.max(1,Math.abs(strafe)+Math.abs(forward)+Math.abs(rotate)) ;
        LF = (strafe+forward+rotate)/denominator*drivepow;
        RF = (-strafe+forward-rotate)/denominator*drivepow;
        LR = (-strafe+forward+rotate)/denominator*drivepow;
        RR = (strafe+forward-rotate)/denominator*drivepow;
        setMotorPowers(LF, LR, RR, RF);
    }

    public void setMotorPowers(double v, double v1, double v2, double v3) {
        drive.leftFront.setPower(v);
        drive.leftRear.setPower(v1);
        drive.rightRear.setPower(v2);
        drive.rightFront.setPower(v3);
    }

    public void updateClaw()
    {
        if(gamepad1.right_bumper) {
            mecanisme.claw.Close();
        }else if(gamepad1.left_bumper){
            mecanisme.claw.Open();
        }
        if(gamepad1.dpad_down){
            mecanisme.lift.fourBar.down();
        }else if(gamepad1.dpad_up) {
            mecanisme.lift.fourBar.up();
        }
        if(gamepad1.dpad_left || gamepad1.dpad_right){
            mecanisme.claw.dropConeAndKeepBeacon();
        }
    }

    public void updateLift() {
        mecanisme.lift.setPower(gamepad1.left_trigger - gamepad1.right_trigger);

        if (gamepad1.a){ // auto control
            mecanisme.lift.goToHigh();
        }else if(gamepad1.y){
            mecanisme.lift.goToMid();
        }else if(gamepad1.b){
            mecanisme.lift.goToLow();
        }else if(gamepad1.x){
            mecanisme.lift.retractFully();
            mecanisme.claw.Open();
        }
    }
    public void updatetelemetry(){
        telemetry.addData("x", drive.getPoseEstimate().getX());
        telemetry.addData("y", drive.getPoseEstimate().getY());
        telemetry.addData("heading", drive.getPoseEstimate().getHeading());
        telemetry.addData("lift", mecanisme.lift.lift.getCurrentPosition());
        telemetry.update();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        mecanisme.lift.fourBar.down();
        mecanisme.claw.Open();
        while(!isStopRequested())
        {
            updateDrivePowers();
            updateClaw();
            updateLift();
            mecanisme.update();
            drive.update();
            updatetelemetry();
        }
    }
}