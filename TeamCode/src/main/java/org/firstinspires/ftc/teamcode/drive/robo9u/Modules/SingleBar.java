package org.firstinspires.ftc.teamcode.drive.robo9u.Modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.robo9u.ModdedHardware.SafeServo;

@Config
public class SingleBar {
    public enum SingleBarState{
        Front,
        Up,
        Back
    }
    public SingleBarState singleBarState = SingleBarState.Front;
    private ElapsedTime timeSinceLastStateChange;
    public SafeServo leftServo, rightServo, wristServo;

    SingleBar(HardwareMap hw){
        leftServo = new SafeServo(hw,"rightLiftServo") ;
        rightServo = new SafeServo(hw,"leftLiftServo") ;
        wristServo = new SafeServo(hw,"wristServo");
        timeSinceLastStateChange = new ElapsedTime();
        timeSinceLastStateChange.reset();
    }

    public void setSingleBarState(SingleBarState state){
        if(singleBarState != state) {
            singleBarState = state;
            timeSinceLastStateChange.reset();
        }
    }

    public void update(){
        switch(singleBarState){
            case Front:
                    leftServo.setPosition(0.68);
                    rightServo.setPosition(0.32);
                    if (timeSinceLastStateChange.milliseconds() > 150)
                        wristServo.setPosition(0.23);
                break;
            case Up:
                    leftServo.setPosition(0.39);
                    rightServo.setPosition(0.61);
                    if (timeSinceLastStateChange.milliseconds() > 150)
                        wristServo.setPosition(0.59);
                break;
            case Back:
                    leftServo.setPosition(0.17);
                    rightServo.setPosition(0.83);
                    if (timeSinceLastStateChange.milliseconds() > 150)
                        wristServo.setPosition(0.95);
                break;
        }
    }

}
