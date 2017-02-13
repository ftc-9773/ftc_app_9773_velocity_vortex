package org.firstinspires.ftc.teamcode.attachments;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FTCRobot;

/**
 * Created by pranavburugula on 2/10/2017.
 */

/*
 * Copyright (c) 2017 Robocracy 9773
 */

public class GamepadHandler {
    LinearOpMode curOpMode;
    FTCRobot robot;

    public GamepadHandler(LinearOpMode curOpMode, FTCRobot robot){
        this.curOpMode = curOpMode;
        this.robot = robot;
    }
}
