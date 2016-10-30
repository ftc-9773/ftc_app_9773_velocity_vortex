package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOpMode", group = "TeleOp")
public class TeleOpMode extends LinearOpMode {

    @Override
    public void runOpMode(){

        waitForStart();
        while(opModeIsActive()){

            idle();
        }
    }
}
