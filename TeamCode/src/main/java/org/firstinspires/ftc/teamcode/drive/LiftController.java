package org.firstinspires.ftc.teamcode.drive;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import java.util.Arrays;
import java.util.List;

public class LiftController {
    private PIDFController individualController;
    private PIDController relativeController;

    private final double ticksPerCM = 0;
    public static double kp = 0, ki = 0, kd = 0, kf = 0, rp = 0, ri = 0, rd = 0;
    public double target = 0; //ticks

    public DcMotorEx left, right;
    private List<DcMotorEx> motors;

    private boolean isBusy;

    public LiftController(HardwareMap hw, boolean resetEncoders) {
        individualController = new PIDFController(kp, ki, kd, kf);
        relativeController = new PIDController(rp, ri, rd);

        left = hw.get(DcMotorEx.class, "liftLeft");
        right = hw.get(DcMotorEx.class, "liftRight");

        motors = Arrays.asList(left, right);

        for(DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        left.setDirection(DcMotorEx.Direction.REVERSE);

        if(!resetEncoders) return;
        for(DcMotorEx motor : motors) {
            motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public boolean isBusy(){
        return isBusy;
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

    public void update(){
        individualController = new PIDFController(kp, ki, kd, kf);
        relativeController.setPID(rp, ri, rd);
        double leftPower, rightPower;
        double motorRelativeError = Math.abs(left.getCurrentPosition()-right.getCurrentPosition())>10?left.getCurrentPosition()-right.getCurrentPosition():0;
        double individualPower = individualController.calculate(getCurrentPosition(), target);
        double relativePower = relativeController.calculate(motorRelativeError, 0);
        leftPower = individualPower-relativePower;
        rightPower = individualPower+relativePower;
        double denom = Math.max(leftPower, Math.max(rightPower, 1));
        left.setPower(leftPower / denom);
        right.setPower(rightPower / denom);
        if(individualController.atSetPoint() && relativeController.atSetPoint()){
            isBusy = false;
        }
    }

    public void setTarget(double cmTarget){
        target = cmTarget * ticksPerCM;
        isBusy = true;
    }

    public double encoderTicksToCM(double ticks) {
        return ticks/ticksPerCM;
    }
}