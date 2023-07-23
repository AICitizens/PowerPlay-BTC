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
        leftServo.setPosition(0.875);
        rightServo.setPosition(0.125);
        wristServo.setPosition(0.20);
    }

    public void Up(){
        leftServo.setPosition(3.0/6);
        rightServo.setPosition(3.0/6);
        wristServo.setPosition(3.0/6);
    }

    public void Back(){
        leftServo.setPosition(0.25);
        rightServo.setPosition(0.75);
        wristServo.setPosition(0.82);
    }

}
