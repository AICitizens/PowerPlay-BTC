package org.firstinspires.ftc.teamcode.drive.robo9u.Modules;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.drive.robo9u.ModdedHardware.SafeMotor;

import java.util.Arrays;
import java.util.List;

public class MecanumDrivetrain {
    private SafeMotor leftFront, rightFront, leftRear, rightRear;
    private List<SafeMotor> motors;
    public MecanumDrivetrain(HardwareMap hardwareMap, boolean reverseMotors) {
        leftFront = new SafeMotor(hardwareMap, "leftFront");
        rightFront = new SafeMotor(hardwareMap, "rightFront");
        leftRear = new SafeMotor(hardwareMap, "leftRear");
        rightRear = new SafeMotor(hardwareMap, "rightRear");

        motors = Arrays.asList(leftFront, rightFront, leftRear, rightRear);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        for (SafeMotor motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }
    }
    public void driveFieldCentric(double forward, double strafe, double rotate, double heading){
        driveRobotCentric( forward*Math.cos(heading)-strafe*Math.sin(heading),
                            forward*Math.sin(heading)+strafe*Math.cos(heading),
                                    rotate);
    }
    public void driveRobotCentric(double forward, double strafe, double rotate){
        double denominator = Math.max(1, forward+strafe+rotate);
        forward = forward/denominator; strafe = strafe/denominator; rotate = rotate/denominator;
        leftFront.setPower(forward+strafe+rotate);
        rightFront.setPower(forward-strafe-rotate);
        leftRear.setPower(forward-strafe+rotate);
        rightRear.setPower(forward+strafe-rotate);
    }
}