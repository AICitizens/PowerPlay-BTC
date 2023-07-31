package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.LiftController;

@TeleOp
public class LiftCalibrate extends LinearOpMode {
    LiftController lift = null;

    @Override
    public void runOpMode() throws InterruptedException {
        lift = new LiftController(hardwareMap);
        waitForStart();
        while(!isStopRequested() && opModeIsActive()){
            lift.update();
            telemetry.addData("pos", lift.getCurrentPosition());
            telemetry.update();
        }
    }
}
