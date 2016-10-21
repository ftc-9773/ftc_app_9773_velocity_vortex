package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutonomousActions;
import org.firstinspires.ftc.teamcode.FTCRobot;
import org.firstinspires.ftc.teamcode.util.JsonReaders.JsonReader;
import org.firstinspires.ftc.teamcode.navigation.LineFollow;
import org.json.JSONException;
import org.json.JSONObject;

@Autonomous(name = "Replay", group = "Autonomous")
public class AutonomousBlue extends LinearOpMode {

    FTCRobot robot;

    @Override
    public void runOpMode() throws InterruptedException{
        JsonReader opmodeCfg = new JsonReader(JsonReader.opModesDir +
                this.getClass().getName() + ".json");
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
        robot.runAutonomous(autonomousOpt, "blue", startingDelay, startingPosition);
    }
}
