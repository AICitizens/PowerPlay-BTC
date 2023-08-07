package org.firstinspires.ftc.teamcode.drive.robo9u.ModdedHardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class SafeMotor {
    public DcMotorEx motor;
    private double motorPower = -1e7;
    public SafeMotor(HardwareMap hardwareMap, String name){
        motor = hardwareMap.get(DcMotorEx.class, name);
    }

    public void setPower(double power){
        if(motorPower != power) {
            motorPower = power;
            motor.setPower(power);
        }
    }

    public double getCurrent(CurrentUnit unit){
        return motor.getCurrent(unit);
    }

    public int getCurrentPosition(){
        return motor.getCurrentPosition();
    }

    public void setMode(DcMotor.RunMode runMode){
        motor.setMode(runMode);
    }

    public void setMotorType(MotorConfigurationType motorType){
        motor.setMotorType(motorType);
    }

    public void setDirection(DcMotorSimple.Direction direction){
        motor.setDirection(direction);
    }

    public MotorConfigurationType getMotorType(){
        return motor.getMotorType();
    }
}
