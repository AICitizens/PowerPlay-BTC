package org.firstinspires.ftc.teamcode.ModdedHardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

public class SafeMotor {
    public DcMotorEx motor;
    private double motorPower = -2;
    public SafeMotor(HardwareMap hardwareMap, String name){
        motor = hardwareMap.get(DcMotorEx.class, name);
    }

    public void setPower(double power){
        if(motorPower != power) {
            motorPower = power;
            motor.setPower(power);
        }
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
