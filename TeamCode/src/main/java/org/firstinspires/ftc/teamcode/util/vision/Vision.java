/*
 * Copyright (c) 2016 Robocracy 9773.
 */

package org.firstinspires.ftc.teamcode.util.vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcontroller.internal.vision.BeaconColorResult;
import org.firstinspires.ftc.robotcontroller.internal.vision.FrameGrabber;
import org.firstinspires.ftc.robotcontroller.internal.vision.ImageProcessorResult;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.FTCRobot;

/**
 * Created by pb8xe_000 on 12/30/2016.
 */

public class Vision {
    FrameGrabber frameGrabber=null;
    LinearOpMode curOpMode=null;
    FTCRobot robot=null;
    VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
    parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

    public Vision(LinearOpMode curOpMode, FTCRobot robot){
        this.curOpMode = curOpMode;
        this.robot = robot;
        this.frameGrabber= FtcRobotControllerActivity.frameGrabber;
    }

    public BeaconColorResult.BeaconColor[] getVisionBeaconColors(){
        frameGrabber.grabSingleFrame();
        while (!frameGrabber.isResultReady() && curOpMode.opModeIsActive()){
            curOpMode.sleep(5);
        }

        ImageProcessorResult imageProcessorResult = frameGrabber.getResult();
        BeaconColorResult result = (BeaconColorResult) imageProcessorResult.getResult();

        BeaconColorResult.BeaconColor leftColor = result.getLeftColor();
        BeaconColorResult.BeaconColor rightColor = result.getRightColor();

        BeaconColorResult.BeaconColor[] results = {leftColor, rightColor};

        curOpMode.telemetry.addData("Beacon color result: ", results.toString());
        curOpMode.telemetry.update();


        return results;
    }
}