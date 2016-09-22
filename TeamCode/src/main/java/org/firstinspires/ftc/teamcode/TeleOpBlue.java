/*
 * Copyright (c) 2016 Robocracy 9773.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOpBlue", group = "TeleOp")
@Disabled
public class TeleOpBlue extends LinearOpMode{

    public void runOpMode() throws InterruptedException{

        waitForStart();
        while (opModeIsActive()){
            // rb's addition 3
            idle();
        }
    }
}
