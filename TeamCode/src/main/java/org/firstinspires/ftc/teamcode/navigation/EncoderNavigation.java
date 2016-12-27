package org.firstinspires.ftc.teamcode.navigation;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FTCRobot;
import org.firstinspires.ftc.teamcode.drivesys.DriveSystem;

/**
 * Created by rsburugula on 12/26/16.
 */

public class EncoderNavigation {
    private double currentYaw = 0.0;
    private FTCRobot robot;
    private DriveSystem driveSys;
    private LinearOpMode curOpMode;
    private Navigation navigation;

    public EncoderNavigation(FTCRobot robot, DriveSystem driveSys, LinearOpMode curOpMode,
                             Navigation navigation, double currentYaw) {
        this.robot = robot;
        this.driveSys = driveSys;
        this.curOpMode = curOpMode;
        this.navigation = navigation;
        this.currentYaw = currentYaw;
    }

    public void driveToDistance(double inches, double speed) {

    }

    public void setRobotOrientation (double targetYaw, double speed, NavigationChecks navExc) {
        double degrees;
        degrees = navigation.getDegreesToTurn(currentYaw, targetYaw);

        driveSys.turnDegrees(degrees, (float)speed, navExc);
    }

    public double getCurrentYaw() {
        return currentYaw;
    }
}
