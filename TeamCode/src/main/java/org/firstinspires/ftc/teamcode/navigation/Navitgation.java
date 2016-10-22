package org.firstinspires.ftc.teamcode.navigation;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FTCRobot;
import org.firstinspires.ftc.teamcode.util.JsonReaders.JsonReader;
import org.firstinspires.ftc.teamcode.util.JsonReaders.NavigationOptionsReader;
import org.json.JSONObject;



public class Navitgation {
    FTCRobot robot;
    LinearOpMode curOpMode;
    JSONObject navOptObj;
    public LineFollow lf;

    public Navitgation(FTCRobot robot, LinearOpMode curOpMode, String navOptionStr) {
        this.robot = robot;
        this.curOpMode = curOpMode;

        NavigationOptionsReader navOption = new NavigationOptionsReader(JsonReader.navigationFile,
                navOptionStr);
        this.navOptObj = navOption.jsonRoot;

        this.lf = new LineFollow(robot, navOption.getLightSensorName(),
                navOption.getLineFollowLowSpeed(), navOption.getLineFollowHighSpeed());
    }
}
