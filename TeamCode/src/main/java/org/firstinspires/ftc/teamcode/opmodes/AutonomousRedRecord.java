package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FTCRobot;
import org.firstinspires.ftc.teamcode.util.JsonReaders.JsonReader;
import org.json.JSONException;

@Autonomous(name = "AutonomousRedRecord", group = "TeleOp")
public class AutonomousRedRecord extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        JsonReader opmodeCfg = new JsonReader(JsonReader.opModesDir +
                this.getClass().getName() + ".json");
        String robotName = null;
        try {
            robotName = opmodeCfg.jsonRoot.getString("robot");
        } catch (JSONException e) {
            e.printStackTrace();
        }
        FTCRobot robot = new FTCRobot(this, robotName);
        robot.autonomousRecord(opmodeCfg, "red");
    }
}
