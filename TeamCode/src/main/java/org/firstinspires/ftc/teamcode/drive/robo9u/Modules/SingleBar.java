package org.firstinspires.ftc.teamcode.drive.robo9u.Modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.ModdedHardware.SafeServo;

@Config
public class SingleBar {
    public SafeServo leftServo, rightServo, wristServo;

    SingleBar(HardwareMap hw){
        leftServo=hw.get(SafeServo.class,"rightLiftServo") ;
        rightServo=hw.get(SafeServo.class,"leftLiftServo") ;
        wristServo=hw.get(SafeServo.class, "wristServo");
    }

    public void Front(){
        leftServo.setPosition(0.68);
        rightServo.setPosition(0.32);
        wristServo.setPosition(0.23);
    }

    public void Up(){
        leftServo.setPosition(0.5);
        rightServo.setPosition(0.5);
        wristServo.setPosition(0.5);
    }

    public void Back(){
        leftServo.setPosition(0.17);
        rightServo.setPosition(0.83);
        wristServo.setPosition(0.90);
    }

}
