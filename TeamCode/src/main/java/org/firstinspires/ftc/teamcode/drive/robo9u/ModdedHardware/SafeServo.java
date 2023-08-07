package org.firstinspires.ftc.teamcode.drive.robo9u.ModdedHardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class SafeServo {
    public Servo servo;
    private double servoPosition = -2;

    public SafeServo(HardwareMap hardwareMap, String name){
        servo = hardwareMap.get(Servo.class, name);
    }

    public void setPosition(double position){
        if(servoPosition != position) {
            servoPosition = position;
            servo.setPosition(position);
        }
    }

    public double getPosition(){
        return servoPosition;
    }
}
