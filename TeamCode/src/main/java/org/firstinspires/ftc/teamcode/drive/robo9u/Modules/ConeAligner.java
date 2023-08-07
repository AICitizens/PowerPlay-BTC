package org.firstinspires.ftc.teamcode.drive.robo9u.Modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.robo9u.ModdedHardware.SafeServo;

@Config
public class ConeAligner {
    public static double down = 0.75,  up = 0.4;
    public SafeServo servo;

    ConeAligner(HardwareMap hw){
        servo = new SafeServo(hw, "ConeAligner");
    }
    public void Down(){
        servo.setPosition(down);
    }
    public void Up(){
        servo.setPosition(up);
    }
}
