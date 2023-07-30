package org.firstinspires.ftc.teamcode.drive.robo9u.Modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.ModdedHardware.SafeServo;

@Config
public class ConeAligner {
    public static double down = 0.4,  up = 0.6;
    SafeServo SafeServo;

    ConeAligner(HardwareMap hw){
        SafeServo = new SafeServo(hw, "Gripper");
    }
    public void Down(){
        SafeServo.setPosition(down);
    }
    public void Up(){
        SafeServo.setPosition(up);
    }
}
