package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
 * Copyright (c) 2016 Robocracy 9773
 */

@TeleOp(name = "TeleOpMode", group = "TeleOp")
@Disabled
public class TeleOpMode extends LinearOpMode {

    @Override
    public void runOpMode(){

        waitForStart();
        while(opModeIsActive()){



            idle();
        }
    }
}
