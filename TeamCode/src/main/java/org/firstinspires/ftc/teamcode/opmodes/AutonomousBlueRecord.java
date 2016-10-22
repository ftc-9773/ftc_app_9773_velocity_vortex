package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FTCRobot;
import org.firstinspires.ftc.teamcode.util.JsonReaders.JsonReader;
import org.json.JSONException;

@Autonomous(name = "AutonomousBlueRecord", group = "TeleOp")
public class AutonomousBlueRecord extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        JsonReader opmodeCfg = new JsonReader(JsonReader.opModesDir +
                "AutonomousBlueRecord.json");
        String robotName = null;
        try {
            robotName = opmodeCfg.jsonRoot.getString("robot");
        } catch (JSONException e) {
            e.printStackTrace();
        }
        FTCRobot robot = new FTCRobot(this, robotName);
        robot.autonomousRecord(opmodeCfg, "blue");
    }
}
