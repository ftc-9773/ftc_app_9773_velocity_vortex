package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FTCRobot;

/**
 * Created by Kids on 12/27/2016.
 */
@Autonomous(name = "test", group = "Autonomous")
public class AutonomousTest extends LinearOpMode {

    FTCRobot robot;

    public void runOpMode(){
        robot = new FTCRobot(this,"robot");

        waitForStart();
        while (opModeIsActive()){
            robot.driveSystem.testEncoders();
            idle();
        }
    }
}
