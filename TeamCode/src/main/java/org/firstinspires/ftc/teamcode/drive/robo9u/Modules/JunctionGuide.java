package org.firstinspires.ftc.teamcode.drive.robo9u.Modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.robo9u.ModdedHardware.SafeServo;

@Config
public class JunctionGuide {
    public static double down = 0.3,  up = 0.6;
    SafeServo SafeServo;

    JunctionGuide(HardwareMap hw){
        SafeServo = new SafeServo(hw, "guideServo");
    }
    public void Down(){
        SafeServo.setPosition(down);
    }
    public void Up(){
        SafeServo.setPosition(up);
    }
}
