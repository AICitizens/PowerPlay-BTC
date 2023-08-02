package org.firstinspires.ftc.teamcode.drive;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.ModdedHardware.SafeMotor;

import java.util.Arrays;
import java.util.List;

@Config
public class LiftController {
    private final PIDFController controller;

    public static final double TICKS_PER_REV = 28*8.3521;
    public static double WHEEL_RADIUS = 1; // cm
    public static double GEAR_RATIO = 1.8163; // output (wheel) speed / input (motor) speed
    public static double kp = 0.013, ki = 0, kd = 0.0005, ff = 0.00004, relativeP = 0.0005;
    public static double target = 0; //ticks
    public static double MAX_TARGET = 2000, MAX_CURRENT = 250;

    private boolean canOverride = true;

    public SafeMotor left, right;
    List<SafeMotor> motors;

    public LiftController(HardwareMap hw) {
        controller = new PIDFController(kp, ki, kd, ff);
        controller.setTolerance(15);

        left = new SafeMotor(hw, "liftLeft");
        right = new SafeMotor(hw, "liftRight");
        motors = Arrays.asList(left, right);

        for(SafeMotor motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        right.setDirection(DcMotorEx.Direction.FORWARD);
        left.setDirection(DcMotorEx.Direction.REVERSE);
    }

    public boolean isBusy(){
        return !canOverride;
    }

    public double getCurrentPosition(){
        return ((float)left.getCurrentPosition()+(float)right.getCurrentPosition())/2;
    }

    public void setPower(double power) {
        left.setPower(power);
        right.setPower(power);
    }

    public void stop(){
        target = getCurrentPosition();
    }

    double lastPos = 0;
    public void update(){
        double deltaPos = getCurrentPosition()-lastPos;
        if(left.getCurrent(CurrentUnit.AMPS)+right.getCurrent(CurrentUnit.AMPS) > MAX_CURRENT) {
            if(deltaPos < 0) {
                target = 0;
                for (SafeMotor motor : motors) {
                    motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                }
            }else {
                MAX_TARGET = target;
                target = getCurrentPosition()-deltaPos;
            }
        }
        controller.setPIDF(kp, ki, kd, ff);
        double leftPower, rightPower;
        double motorRelativeError = Math.abs(left.getCurrentPosition()-right.getCurrentPosition())>10?left.getCurrentPosition()-right.getCurrentPosition():0;
        double power = controller.calculate(getCurrentPosition(), target);
        leftPower = power-relativeP*motorRelativeError;
        rightPower = power+relativeP*motorRelativeError;
        double denom = Math.max(leftPower, Math.max(rightPower, 1));
        left.setPower(leftPower / denom);
        right.setPower(rightPower / denom);
        if(controller.atSetPoint()){
            canOverride = true;
        }
    }

    public void setTarget(double newTarget){
        target = Math.min(newTarget * TICKS_PER_REV / WHEEL_RADIUS / 2 / Math.PI / GEAR_RATIO, MAX_TARGET);
        canOverride = false;
    }

    public double encoderTicksToCM(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }
}
