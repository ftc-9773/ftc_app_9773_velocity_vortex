package org.firstinspires.ftc.teamcode.util;

//import android.support.annotation.TransitionRes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by michaelzhou on 12/30/16.
 */

public class StateMachine {
    private enum State{
       TeleOp_ShootParticles,
        EndGame_CapBall,
        TeleOp_BeaconClaim;
    }

    State state;
    ElapsedTime timer;

    public StateMachine(){
        state = State.TeleOp_ShootParticles;
        timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    }

    public void transition(OpMode opMode){
        if(opMode.gamepad1.back){
            switch (state){
                case TeleOp_BeaconClaim: state = timer.seconds()>120-40 ? State.EndGame_CapBall : State.TeleOp_ShootParticles; break;
                case EndGame_CapBall: state = State.TeleOp_ShootParticles; break;
                case TeleOp_ShootParticles: state = State.TeleOp_BeaconClaim; break;
            }
            opMode.telemetry.addData("Current state: ","%s",state.name());
        }
    }

    public State getCurrentState(){return state;}

    public void initAfterStart(){
        this.timer.reset();
    }
}
