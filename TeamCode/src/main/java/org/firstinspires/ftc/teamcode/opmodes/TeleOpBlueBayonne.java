package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.FTCRobot;
import org.firstinspires.ftc.teamcode.util.JsonReaders.JsonReader;
import org.json.JSONException;

@TeleOp(name = "TeleOpBlueBayonne", group = "TeleOp")
public class TeleOpBlueBayonne extends LinearOpMode {
    @Override
    public void runOpMode(){
        FTCRobot robot;
        JsonReader opmodeCfg = new JsonReader(JsonReader.opModesDir +
                this.getClass().getName() + ".json");
        String robotName = null;

        try {
            robotName = opmodeCfg.jsonRoot.getString("robot");
        } catch (JSONException e) {
            e.printStackTrace();
        }

        robot = new FTCRobot(this, robotName);
        robot.runTeleOp("blue");
    }
}
