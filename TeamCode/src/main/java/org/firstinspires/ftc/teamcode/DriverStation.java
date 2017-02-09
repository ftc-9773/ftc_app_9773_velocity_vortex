package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by ftcrobocracy on 2/8/17.
 */

public class DriverStation {
    private LinearOpMode curOpMode;

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
            case "gamepad1.left_bumper" : buttonID = DSGamePad.GAMEPAD1_LEFT_BUMPER; break;
            case "gamepad1.right_bumper" : buttonID = DSGamePad.GAMEPAD1_RIGHT_BUMPER; break;
            case "gamepad1.left_stick_button" : buttonID = DSGamePad.GAMEPAD1_LEFTSTICK_BUTTON; break;
            case "gamepad1.right_stick_button" : buttonID = DSGamePad.GAMEPAD1_RIGHTSTICK_BUTTON; break;
            case "gamepad1.left_trigger" : buttonID = DSGamePad.GAMEPAD1_LEFT_TRIGGER; break;
            case "gamepad1.right_trigger" : buttonID = DSGamePad.GAMEPAD1_RIGHT_TRIGGER; break;
            case "gamepad1.left_stick_x" : buttonID = DSGamePad.GAMEPAD1_LEFTSTICK_X; break;
            case "gamepad1.left_stick_y" : buttonID = DSGamePad.GAMEPAD1_LEFTSTICK_Y; break;
            case "gamepad1.right_stick_x" : buttonID = DSGamePad.GAMEPAD1_RIGHTSTICK_X; break;
            case "gamepad1.right_stick_y" : buttonID = DSGamePad.GAMEPAD1_RIGHTSTICK_Y; break;
            case "gamepad1.back" :  buttonID = DSGamePad.GAMEPAD1_BACK; break;
            case "gamepad2.start" :  buttonID = DSGamePad.GAMEPAD2_START; break;
            case "gamepad2.x" : buttonID = DSGamePad.GAMEPAD2_X; break;
            case "gamepad2.y" : buttonID = DSGamePad.GAMEPAD2_Y; break;
            case "gamepad2.a" : buttonID = DSGamePad.GAMEPAD2_A; break;
            case "gamepad2.b" : buttonID = DSGamePad.GAMEPAD2_B; break;
            case "gamepad2.dpad_up" : buttonID = DSGamePad.GAMEPAD2_DPAD_UP; break;
            case "gamepad2.dpad_down" : buttonID = DSGamePad.GAMEPAD2_DPAD_DOWN; break;
            case "gamepad2.dpad_left" : buttonID = DSGamePad.GAMEPAD2_DPAD_LEFT; break;
            case "gamepad2.dpad_right" : buttonID = DSGamePad.GAMEPAD2_DPAD_RIGHT; break;
            case "gamepad2.left_bumper" : buttonID = DSGamePad.GAMEPAD2_LEFT_BUMPER; break;
            case "gamepad2.right_bumper" : buttonID = DSGamePad.GAMEPAD2_RIGHT_BUMPER; break;
            case "gamepad2.left_stick_button" : buttonID = DSGamePad.GAMEPAD2_LEFTSTICK_BUTTON; break;
            case "gamepad2.right_stick_button" : buttonID = DSGamePad.GAMEPAD2_RIGHTSTICK_BUTTON; break;
            case "gamepad2.left_trigger" : buttonID = DSGamePad.GAMEPAD2_LEFT_TRIGGER; break;
            case "gamepad2.right_trigger" : buttonID = DSGamePad.GAMEPAD2_RIGHT_TRIGGER; break;
            case "gamepad2.left_stick_x" : buttonID = DSGamePad.GAMEPAD2_LEFTSTICK_X; break;
            case "gamepad2.left_stick_y" : buttonID = DSGamePad.GAMEPAD2_LEFTSTICK_Y; break;
            case "gamepad2.right_stick_x" : buttonID = DSGamePad.GAMEPAD2_RIGHTSTICK_X; break;
            case "gamepad2.right_stick_y" : buttonID = DSGamePad.GAMEPAD2_RIGHTSTICK_Y; break;
            case "gamepad2.back" :  buttonID = DSGamePad.GAMEPAD2_BACK; break;
            default: buttonID = DSGamePad.NONE;
        }
        return buttonID;
    }

    public boolean getBoolean(DSGamePad key) {
        boolean value = false;
        switch (key) {
            case GAMEPAD1_START: value = curOpMode.gamepad1.start; break;
            case GAMEPAD1_A: value = curOpMode.gamepad1.a; break;
            case GAMEPAD1_B: value = curOpMode.gamepad1.b; break;
            case GAMEPAD1_X: value = curOpMode.gamepad1.x; break;
            case GAMEPAD1_Y: value = curOpMode.gamepad1.y; break;
            case GAMEPAD1_DPAD_UP: value = curOpMode.gamepad1.dpad_up; break;
            case GAMEPAD1_DPAD_DOWN: value = curOpMode.gamepad1.dpad_down; break;
            case GAMEPAD1_DPAD_LEFT: value = curOpMode.gamepad1.dpad_left; break;
            case GAMEPAD1_DPAD_RIGHT: value = curOpMode.gamepad1.dpad_right; break;
            case GAMEPAD1_LEFT_BUMPER: value=curOpMode.gamepad1.left_bumper; break;
            case GAMEPAD1_RIGHT_BUMPER: value = curOpMode.gamepad1.right_bumper; break;
            case GAMEPAD1_LEFTSTICK_BUTTON: value = curOpMode.gamepad1.left_stick_button; break;
            case GAMEPAD1_RIGHTSTICK_BUTTON: value = curOpMode.gamepad1.right_stick_button; break;
            case GAMEPAD1_BACK: value = curOpMode.gamepad1.back; break;
            case GAMEPAD2_START: value = curOpMode.gamepad2.start; break;
            case GAMEPAD2_A: value = curOpMode.gamepad2.a; break;
            case GAMEPAD2_B: value = curOpMode.gamepad2.b; break;
            case GAMEPAD2_X: value = curOpMode.gamepad2.x; break;
            case GAMEPAD2_Y: value = curOpMode.gamepad2.y; break;
            case GAMEPAD2_DPAD_UP: value = curOpMode.gamepad2.dpad_up; break;
            case GAMEPAD2_DPAD_DOWN: value = curOpMode.gamepad2.dpad_down; break;
            case GAMEPAD2_DPAD_LEFT: value = curOpMode.gamepad2.dpad_left; break;
            case GAMEPAD2_DPAD_RIGHT: value = curOpMode.gamepad2.dpad_right; break;
            case GAMEPAD2_LEFT_BUMPER: value=curOpMode.gamepad2.left_bumper; break;
            case GAMEPAD2_RIGHT_BUMPER: value = curOpMode.gamepad2.right_bumper; break;
            case GAMEPAD2_LEFTSTICK_BUTTON: value = curOpMode.gamepad2.left_stick_button; break;
            case GAMEPAD2_RIGHTSTICK_BUTTON: value = curOpMode.gamepad2.right_stick_button; break;
            case GAMEPAD2_BACK: value = curOpMode.gamepad2.back; break;
        }
        return (value);
    }

    public float getFloat(DSGamePad key) {
        float value = 0.0f;
        switch (key) {
            case GAMEPAD1_LEFTSTICK_X: value = curOpMode.gamepad1.left_stick_x; break;
            case GAMEPAD1_LEFTSTICK_Y: value = curOpMode.gamepad1.left_stick_y; break;
            case GAMEPAD1_LEFT_TRIGGER: value = curOpMode.gamepad1.left_trigger; break;
            case GAMEPAD1_RIGHTSTICK_X: value = curOpMode.gamepad1.right_stick_x; break;
            case GAMEPAD1_RIGHTSTICK_Y: value = curOpMode.gamepad1.right_stick_y; break;
            case GAMEPAD1_RIGHT_TRIGGER: value = curOpMode.gamepad1.right_trigger; break;
            case GAMEPAD2_LEFTSTICK_X: value = curOpMode.gamepad2.left_stick_x; break;
            case GAMEPAD2_LEFTSTICK_Y: value = curOpMode.gamepad2.left_stick_y; break;
            case GAMEPAD2_LEFT_TRIGGER: value = curOpMode.gamepad2.left_trigger; break;
            case GAMEPAD2_RIGHTSTICK_X: value = curOpMode.gamepad2.right_stick_x; break;
            case GAMEPAD2_RIGHTSTICK_Y: value = curOpMode.gamepad2.right_stick_y; break;
            case GAMEPAD2_RIGHT_TRIGGER: value = curOpMode.gamepad2.right_trigger; break;
            default: value=0.0f;
        }
        return (value);
    }

    public DriverStation(LinearOpMode curOpMode) {
        this.curOpMode = curOpMode;
    }
}
