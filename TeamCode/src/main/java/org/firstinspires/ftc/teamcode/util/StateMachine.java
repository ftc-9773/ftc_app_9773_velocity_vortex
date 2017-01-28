package org.firstinspires.ftc.teamcode.util;

//import android.support.annotation.TransitionRes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

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

    public StateMachine(){
        this.state = State.TeleOp_ShootParticles;
    }

    public void transition(OpMode opMode){
        if(opMode.gamepad1.back){
            switch (state){
                case TeleOp_BeaconClaim: state = State.EndGame_CapBall;
                    opMode.telemetry.addData("Current State: ","Cap Ball");
                    break;
                case EndGame_CapBall: state = State.TeleOp_ShootParticles;
                    opMode.telemetry.addData("Current State: ","Shoot Particles");
                    break;
                case TeleOp_ShootParticles: state = State.TeleOp_BeaconClaim;
                    opMode.telemetry.addData("Current State: ","Beacon Claim");
                    break;
            }
        }

    }

    public State getCurrentState(){return state;}

}
