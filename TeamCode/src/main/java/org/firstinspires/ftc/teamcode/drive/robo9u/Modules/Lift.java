package org.firstinspires.ftc.teamcode.drive.robo9u.Modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.drive.LiftController;

@Config
public class Lift {
    public enum LiftState{
        High,
        Mid,
        Low,
        Ground,
        Defence,
        Stack,
        Idle,
    }
    public LiftState liftState = LiftState.Idle;

    private ElapsedTime singlebarTimer;
    public LiftController lift;
    public SingleBar singleBar;
    public static double ground = 1, low = 17, mid = 43 , high = 69, stackConeDist = 3.55, stackPos;

    private boolean manualControl = false;

    public void stopCurrentTrajectory(){
        lift.stop();
    }

    public void setLiftState(LiftState state){
        liftState = state;
        singlebarTimer.reset();
    }

    public void setPower(double power){
        manualControl = false;
        if (lift.isBusy()) return; // nu interfera cu traiectorii
        if(power==0) return;
        liftState = LiftState.Idle;
        manualControl = true;
        lift.setPower(lift.getCurrentPosition()<5?Math.max(power, 0):power);
        lift.stop();
    }

    public void nextStack(){
        if(stackPos == 0)
            stackPos = 5;
        stackPos -=1;
        liftState = LiftState.Stack;
    }

    public void update(){
        switch (liftState){
            case High:
                lift.setTarget(high);
                singleBar.Back();
                break;
            case Mid:
                lift.setTarget(mid);
                singleBar.Back();
                break;
            case Low:
                lift.setTarget(low);
                singleBar.Back();
                break;
            case Ground:
                lift.setTarget(ground);
                singleBar.Front();
                break;
            case Defence:
                lift.setTarget(ground);
                singleBar.Up();
                break;
            case Stack:
                lift.setTarget(stackConeDist*stackPos + 0.2+ground);
                singleBar.Front();
                break;
            case Idle:
                break;
        }
        if(!manualControl)
            lift.update();
    }

    public Lift(HardwareMap hw){
        lift = new LiftController(hw);
        singleBar = new SingleBar(hw);
        stackPos = 5;
        singlebarTimer = new ElapsedTime();
        singlebarTimer.reset();
    }
}
