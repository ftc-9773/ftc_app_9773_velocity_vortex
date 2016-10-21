package org.firstinspires.ftc.teamcode.navigation;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FTCRobot;
import org.firstinspires.ftc.teamcode.util.JsonReaders.NavigationOptionsReader;
import org.json.JSONObject;



public class Navitgation {
    String navigationConfigFile;
    FTCRobot robot;
    public LineFollow lf;
    LinearOpMode curOpMode;

    public Navitgation(FTCRobot robot, LinearOpMode curOpMode, String navigationConfigFile) {
        this.robot = robot;
        this.curOpMode = curOpMode;
        this.navigationConfigFile = navigationConfigFile;

        NavigationOptionsReader navOption = new NavigationOptionsReader(navigationConfigFile);

        this.lf = new LineFollow(robot, navOption.getLightSensorName(),
                navOption.getLineFollowLowSpeed(), navOption.getLineFollowHighSpeed());
    }
}
