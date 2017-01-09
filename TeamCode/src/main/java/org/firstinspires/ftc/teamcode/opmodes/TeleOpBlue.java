package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.FTCRobot;
import org.firstinspires.ftc.teamcode.util.JsonReaders.JsonReader;
import org.json.JSONException;

/*
 * Copyright (c) 2016 Robocracy 9773
 */

@TeleOp(name = "TeleOpBlue", group = "TeleOp")
public class TeleOpBlue extends LinearOpMode {
    @Override
    public void runOpMode(){
        FTCRobot robot;
        JsonReader opmodeCfg = new JsonReader(JsonReader.opModesDir +
                "TeleOpBlue.json");
        String robotName = null;

        try {
            robotName = opmodeCfg.jsonRoot.getString("robot");
        } catch (JSONException e) {
            e.printStackTrace();
        }

        // TODO: 12/31/16 Instead of passing a 3rd parameter Autonomous/Teleop, use annotations to detect
        //     that FTCRobot is being instantiated for Autonomous or Teleop mode
        robot = new FTCRobot(this, robotName, "Teleop");
        robot.runTeleOp("blue");
    }
}
