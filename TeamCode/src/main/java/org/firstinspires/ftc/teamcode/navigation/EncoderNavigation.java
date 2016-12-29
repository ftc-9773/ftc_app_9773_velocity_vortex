package org.firstinspires.ftc.teamcode.navigation;

import com.qualcomm.ftccommon.DbgLog;
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
                             Navigation navigation) {
        this.robot = robot;
        this.driveSys = driveSys;
        this.curOpMode = curOpMode;
        this.navigation = navigation;
    }

    public void updateCurrentYaw(double degreesTurned){
        currentYaw += degreesTurned;
        if (currentYaw < 0){
            currentYaw += 360;
        }
        else if (currentYaw > 360){
            currentYaw -= 360;
        }
    }

    public void driveToDistance(double inches, double speed) {

    }

    public void setRobotOrientation (double targetYaw, double speed, NavigationChecks navExc) {
        double degrees = navigation.getDegreesToTurn(currentYaw, targetYaw);

        DbgLog.msg("degrees: %f, currYaw: %f, targetYaw: %f", degrees, this.getCurrentYaw(), targetYaw);

        driveSys.turnDegrees(degrees, (float)speed, navExc);
        this.updateCurrentYaw(degrees);
    }

    public void shiftRobot(double distance, boolean isForward, NavigationChecks navigationChecks){
        double moveDistance = Math.sqrt(100 + Math.pow(Math.abs(distance), 2));
        double angle = 90 - Math.toDegrees(Math.asin(10/moveDistance));

        if (isForward){
            if (distance < 0) {
                angle *= -1;
            }
            driveSys.turnDegrees(angle, 0.5f, navigationChecks);
            driveSys.driveToDistance(0.5f, moveDistance);
            driveSys.turnDegrees(-angle, 0.5f, navigationChecks);
            driveSys.driveToDistance(0.5f, -moveDistance);
        }
        else{
            if (distance > 0){
                angle *= -1;
            }
            driveSys.turnDegrees(angle, 0.5f, navigationChecks);
            driveSys.driveToDistance(0.5f, -moveDistance);
            driveSys.turnDegrees(-angle, 0.5f, navigationChecks);
            driveSys.driveToDistance(0.5f, moveDistance);
        }
    }


    public double getCurrentYaw() {
        return currentYaw;
    }
}
