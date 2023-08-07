package org.firstinspires.ftc.teamcode.drive.robo9u.Modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.robo9u.ModdedHardware.SafeServo;

@Config
public class Claw {
    public static double closedPosition = 0.44,  openPosition = 0.275;

    public SafeServo safeServo;

    Claw(HardwareMap hw){
        safeServo = new SafeServo(hw, "Gripper");
    }
    public void Close(){
        safeServo.setPosition(closedPosition);
    }
    public void Open(){
        safeServo.setPosition(openPosition);
    }
}
