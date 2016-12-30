package org.firstinspires.ftc.teamcode.util;

import android.support.annotation.TransitionRes;

/**
 * Created by michaelzhou on 12/30/16.
 */

public class StateMachine {
    private enum State{
        STRING_GAMESTATE1,
        STRING_GAMESTATE2;
    }

    State state;

    void transition(String input){
        switch(input){
            case "gamePad1": setState(State.STRING_GAMESTATE1); break;
            case "gamePad2": setState(State.STRING_GAMESTATE2); break;
        }
    }

    void setState(State newState){
        state = newState;
    }
}
