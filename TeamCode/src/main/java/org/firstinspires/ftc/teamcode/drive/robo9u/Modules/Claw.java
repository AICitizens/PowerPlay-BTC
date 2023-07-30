package org.firstinspires.ftc.teamcode.drive.robo9u.Modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.ModdedHardware.SafeServo;

@Config
public class Claw {
    public static double closedPosition = 0.44,  openPosition = 0.275;

    SafeServo SafeServo;

    Claw(HardwareMap hw){
        SafeServo = new SafeServo(hw, "Gripper");
    }
    public void Close(){
        SafeServo.setPosition(closedPosition);
    }
    public void Open(){
        SafeServo.setPosition(openPosition);
    }
}
