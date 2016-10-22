package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FTCRobot;
import org.firstinspires.ftc.teamcode.util.JsonReaders.JsonReader;
import org.json.JSONException;

@Autonomous(name = "AutonomousRed", group = "Autonomous")
public class AutonomousRed extends LinearOpMode {

    FTCRobot robot;

    @Override
    public void runOpMode() throws InterruptedException{
        JsonReader opmodeCfg = new JsonReader(JsonReader.opModesDir + "AutonomousRed.json");
        String autonomousOpt = null;
        String robotName = null;
        long startingDelay = 0;
        int startingPosition = 1;
        try {
            autonomousOpt = opmodeCfg.jsonRoot.getString("autonomousOption");
            robotName = opmodeCfg.jsonRoot.getString("robot");
            startingDelay = opmodeCfg.jsonRoot.getLong("startingDelay");
            startingPosition = opmodeCfg.jsonRoot.getInt("startingPosition");
        } catch (JSONException e) {
            e.printStackTrace();
        }

        robot = new FTCRobot(this, robotName);
        robot.runAutonomous(autonomousOpt, "red", startingDelay, startingPosition);
    }
}
