package org.firstinspires.ftc.teamcode;

/**
 * Created by ftcrobocracy on 2/8/17.
 */

public class DriverStation {
    public enum DSGamePad {
        GAMEPAD1_START,
        GAMEPAD1_X,
        GAMEPAD1_Y,
        GAMEPAD1_A,
        GAMEPAD1_B,
        GAMEPAD1_DPAD_UP,
        GAMEPAD1_DPAD_DOWN,
        GAMEPAD1_DPAD_LEFT,
        GAMEPAD1_DPAD_RIGHT,
        GAMEPAD1_LEFT_BUMPER,
        GAMEPAD1_RIGHT_BUMPER,
        GAMEPAD1_LEFTSTICK_BUTTON,
        GAMEPAD1_RIGHTSTICK_BUTTON,
        GAMEPAD1_LEFT_TRIGGER,
        GAMEPAD1_RIGHT_TRIGGER,
        GAMEPAD1_LEFTSTICK_X,
        GAMEPAD1_LEFTSTICK_Y,
        GAMEPAD1_RIGHTSTICK_X,
        GAMEPAD1_RIGHTSTICK_Y,
        GAMEPAD1_BACK,
        GAMEPAD2_START,
        GAMEPAD2_X,
        GAMEPAD2_Y,
        GAMEPAD2_A,
        GAMEPAD2_B,
        GAMEPAD2_DPAD_UP,
        GAMEPAD2_DPAD_DOWN,
        GAMEPAD2_DPAD_LEFT,
        GAMEPAD2_DPAD_RIGHT,
        GAMEPAD2_LEFT_BUMPER,
        GAMEPAD2_RIGHT_BUMPER,
        GAMEPAD2_LEFTSTICK_BUTTON,
        GAMEPAD2_RIGHTSTICK_BUTTON,
        GAMEPAD2_LEFT_TRIGGER,
        GAMEPAD2_RIGHT_TRIGGER,
        GAMEPAD2_LEFTSTICK_X,
        GAMEPAD2_LEFTSTICK_Y,
        GAMEPAD2_RIGHTSTICK_X,
        GAMEPAD2_RIGHTSTICK_Y,
        GAMEPAD2_BACK,
        NONE
    }

    public DSGamePad StringToGamepadID(String buttonName) {
        DSGamePad buttonID=DSGamePad.NONE;
        switch (buttonName) {
            case "gamepad1.start" :  buttonID = DSGamePad.GAMEPAD1_START; break;
            case "gamepad1.x" : buttonID = DSGamePad.GAMEPAD1_X; break;
            case "gamepad1.y" : buttonID = DSGamePad.GAMEPAD1_Y; break;
            case "gamepad1.a" : buttonID = DSGamePad.GAMEPAD1_A; break;
            case "gamepad1.b" : buttonID = DSGamePad.GAMEPAD1_B; break;
            case "gamepad1.dpad_up" : buttonID = DSGamePad.GAMEPAD1_DPAD_UP; break;
            case "gamepad1.dpad_down" : buttonID = DSGamePad.GAMEPAD1_DPAD_DOWN; break;
            case "gamepad1.dpad_left" : buttonID = DSGamePad.GAMEPAD1_DPAD_LEFT; break;
            case "gamepad1.dpad_right" : buttonID = DSGamePad.GAMEPAD1_DPAD_RIGHT; break;
        }
        return buttonID;
    }
}
