package org.firstinspires.ftc.teamcode.drive.robo9u.Modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Mechanisms {
    public Lift lift;
    public ConeAligner coneAligner;

    public void update(){
        lift.update();
    }

    public Mechanisms(HardwareMap hw)
    {
        coneAligner = new ConeAligner(hw);
        lift = new Lift(hw);
        coneAligner.Up();
    }
}