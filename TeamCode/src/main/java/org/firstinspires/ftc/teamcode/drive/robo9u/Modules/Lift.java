package org.firstinspires.ftc.teamcode.drive.robo9u.Modules;

import com.acmerobotics.dashboard.FtcDashboard;
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
    private ElapsedTime timeSinceLastState = new ElapsedTime();
    public Claw claw;
    public static double ground = 0.25, low = 16.5, mid = 42 , high = 67, stackConeDist = 3.55, stackPos, lastTarget, dropBy = 0;

    private boolean manualControl = false;

    public void stopCurrentTrajectory(){
        lift.stop();
    }

    public void setLiftState(LiftState state){
        if(liftState == LiftState.High || liftState == LiftState.Mid || liftState == LiftState.Low){
            lastTarget = lift.encoderTicksToCM(lift.getCurrentPosition());
        }

        if(state == LiftState.Ground || state == LiftState.Stack){
            claw.Open();
        }
        timeSinceLastState.reset();
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
        setLiftState(LiftState.Stack);
    }

    public void dropByCM(double cm){
        dropBy = cm;
    }

    public void update(){
        switch (liftState){
            case High:
                if(timeSinceLastState.milliseconds() > 100)
                    lift.setTarget(high - dropBy);
                singleBar.setSingleBarState(SingleBar.SingleBarState.Back);
                break;
            case Mid:
                if(timeSinceLastState.milliseconds() > 100)
                    lift.setTarget(mid - dropBy);
                singleBar.setSingleBarState(SingleBar.SingleBarState.Back);
                break;
            case Low:
                if(timeSinceLastState.milliseconds() > 100)
                    lift.setTarget(low - dropBy);
                singleBar.setSingleBarState(SingleBar.SingleBarState.Back);
                break;
            case Ground:
                dropBy = 0;
                lift.setTarget(ground);
                singleBar.setSingleBarState(SingleBar.SingleBarState.Front);
                break;
            case Defence:
                dropBy = 0;
                claw.Close();
                lift.setTarget(ground);
                singleBar.setSingleBarState(SingleBar.SingleBarState.Up);
                break;
            case Stack:
                dropBy = 0;
                lift.setTarget(stackConeDist*stackPos + 0.2+ground);
                singleBar.setSingleBarState(SingleBar.SingleBarState.Front);
                break;
            case Idle:
                break;
        }
        if(!manualControl)
            lift.update();
        singleBar.update();
    }

    public Lift(HardwareMap hw){
        lift = new LiftController(hw);
        singleBar = new SingleBar(hw);
        claw = new Claw(hw);
        stackPos = 5;
        singlebarTimer = new ElapsedTime();
        singlebarTimer.reset();
    }
}
