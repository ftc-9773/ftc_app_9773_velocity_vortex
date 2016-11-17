package org.firstinspires.ftc.teamcode.navigation;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FTCRobot;
import org.firstinspires.ftc.teamcode.util.JsonReaders.JsonReader;
import org.firstinspires.ftc.teamcode.util.JsonReaders.NavigationOptionsReader;
import org.json.JSONObject;



public class Navigation {
    FTCRobot robot;
    LinearOpMode curOpMode;
    JSONObject navOptObj;
    public LineFollow lf;
    public NavxMicro navxMicro;
    public ModernRoboticsI2cRangeSensor rangeSensor;
    public double minDistance=15.0; // in cm
    public double lfMaxSpeed=1.0, straightDrMaxSpeed=1.0, turnMaxSpeed=1.0;

    public Navigation(FTCRobot robot, LinearOpMode curOpMode, String navOptionStr) {
        this.robot = robot;
        this.curOpMode = curOpMode;

        NavigationOptionsReader navOption = new NavigationOptionsReader(JsonReader.navigationFile,
                navOptionStr);
        this.navOptObj = navOption.jsonRoot;

        if (navOption.lineFollowerExists()) {
            this.lf = new LineFollow(robot, navOption.getLightSensorName(),
                    navOption.getLFvariableDouble("lowSpeed"),
                    navOption.getLFvariableDouble("highSpeed"),
                    navOption.getLFvariableDouble("timeOut"),
                    navOption.getLFvariableDouble("white"),
                    navOption.getLFvariableDouble("black"));
        }
        else {
            this.lf = null;
        }

        if (navOption.imuExists()) {
            this.navxMicro = new NavxMicro(curOpMode, robot, navOption.getIMUDIMname(),
                    navOption.getIMUportNum(), navOption.getIMUdriveSysInitialPower(), navOption.getIMUdriveSysTargetPower(), navOption.getIMUAngleTolerance());
        }
        else {
            this.navxMicro = null;
        }

        if (navOption.rangeSensorExists()) {
            this.rangeSensor = curOpMode.hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeSensor1");
        }
         else {
            this.rangeSensor = null;
        }

        if (navOption.encoderVarsExist()) {
            this.lfMaxSpeed = navOption.getLineFollowMaxSpeed();
            this.straightDrMaxSpeed = navOption.getStraightLineMaxSpeed();
            this.turnMaxSpeed = navOption.getTurningMaxSpeed();
        }
    }
}
