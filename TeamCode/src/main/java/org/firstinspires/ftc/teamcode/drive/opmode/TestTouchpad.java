package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.LiftController;

public class TestTouchpad extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        LiftController lift = new LiftController(hardwareMap);
        Vector2d touchPos = new Vector2d();
        Vector2d relativePos = new Vector2d();
        double liftpos = 0;
        boolean lastTouchpad = false;
        GamepadEx gamepad = new GamepadEx(gamepad1);
        waitForStart();
        while(opModeIsActive() && !isStopRequested()){
            if(gamepad.gamepad.touchpad){
                if(!lastTouchpad){
                    liftpos = lift.getCurrentPosition();
                    touchPos = new Vector2d(gamepad.gamepad.touchpad_finger_1_x, gamepad.gamepad.touchpad_finger_1_y);
                }
                relativePos = new Vector2d(gamepad.gamepad.touchpad_finger_1_x, gamepad.gamepad.touchpad_finger_1_y).minus(touchPos);
                lift.setTarget(liftpos+relativePos.getY()/10+ relativePos.getY()/100);
            }
            telemetry.addData("pos", relativePos);
            telemetry.update();
            lift.update();
        }
    }
}
