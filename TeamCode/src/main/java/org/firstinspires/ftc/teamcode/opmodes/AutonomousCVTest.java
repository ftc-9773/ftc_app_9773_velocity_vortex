package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcontroller.internal.vision.BeaconColorResult;
import org.firstinspires.ftc.robotcontroller.internal.vision.FrameGrabber;
import org.firstinspires.ftc.robotcontroller.internal.vision.ImageProcessorResult;

/**
 * Created by pranavburugula on 12/30/2016.
 */

/*
 * Copyright (c) 2016 Robocracy 9773
 */

@Autonomous(name = "AutonomousCVTest", group = "Autonomous")
public class AutonomousCVTest extends LinearOpMode {

    @Override
    public void runOpMode(){
        waitForStart();
        while (opModeIsActive()){
            FrameGrabber frameGrabber = FtcRobotControllerActivity.frameGrabber; //Get the frameGrabber
            frameGrabber.grabSingleFrame(); //Tell it to grab a frame
            while (!frameGrabber.isResultReady()) { //Wait for the result
                sleep(5); //sleep for 5 milliseconds
            }
//Get the result
            ImageProcessorResult imageProcessorResult = frameGrabber.getResult();
            BeaconColorResult result = (BeaconColorResult) imageProcessorResult.getResult();
            BeaconColorResult.BeaconColor leftColor = result.getLeftColor();
            BeaconColorResult.BeaconColor rightColor = result.getRightColor();
            telemetry.addData("Result", result); //Display it on telemetry
            telemetry.update();
//wait before quitting (quitting clears telemetry)
            sleep(1000);
        }
    }
}
