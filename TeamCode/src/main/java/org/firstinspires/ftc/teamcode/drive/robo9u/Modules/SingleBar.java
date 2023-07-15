package org.firstinspires.ftc.teamcode.drive.robo9u.Modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class SingleBar {
    public static double up = 0.26;
    public static double down = 0.02;
    public Servo leftServo, rightServo, wristServo;

    SingleBar(HardwareMap hw){
        leftServo=hw.get(Servo.class,"rightLiftServo") ;
        rightServo=hw.get(Servo.class,"leftLiftServo") ;
        wristServo=hw.get(Servo.class, "wristServo");
    }

    public void Front(){
        leftServo.setPosition(0);
        rightServo.setPosition(1);
        wristServo.setPosition(0);
    }

    public void Up(){
        leftServo.setPosition(0.5);
        rightServo.setPosition(0.5);
        wristServo.setPosition(0.5);
    }

    public void Back(){
        leftServo.setPosition(1);
        rightServo.setPosition(0);
        wristServo.setPosition(1);
    }

}
