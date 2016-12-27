package org.firstinspires.ftc.teamcode.navigation;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.FTCRobot;
import org.firstinspires.ftc.teamcode.util.JsonReaders.JsonReader;
import org.firstinspires.ftc.teamcode.util.JsonReaders.NavigationOptionsReader;
import org.json.JSONObject;



/*
 * Copyright (c) 2016 Robocracy 9773
 */

public class Navigation {
    FTCRobot robot;
    LinearOpMode curOpMode;
    JSONObject navOptObj;
    public LineFollow lf;
    public NavxMicro navxMicro;
    public EncoderNavigation encoderNav;
    public ModernRoboticsI2cRangeSensor rangeSensor;
    public double minDistance=15.0; // in cm
    public double lfMaxSpeed=1.0, straightDrMaxSpeed=1.0, turnMaxSpeed=1.0;
    public double driveSysTeleopMaxSpeed=1.0;
    public enum SpinDirection {CLOCKWISE, COUNTERCLOCKWISE, NONE}

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
            this.navxMicro = new NavxMicro(curOpMode, robot, this, navOption.getIMUDIMname(),
                    navOption.getIMUportNum(), navOption.getIMUVariableDouble("driveSysInitialPower"),
                    navOption.getIMUVariableDouble("angleTolerance"), navOption.getIMUVariableDouble("straightPID_kp"),
                    navOption.getIMUVariableDouble("turnPID_kp"), navOption.getIMUVariableDouble("PID_minSpeed"),
                    navOption.getIMUVariableDouble("PID_maxSpeed"));
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
            this.driveSysTeleopMaxSpeed = navOption.getDoubleDriveSysEncVar("DriveSysTeleOpMaxSpeed");
        }

        this.encoderNav = new EncoderNavigation(robot, robot.driveSystem, curOpMode, this, 0.0);
    }

    /**
     * Initialize the navigation system just after pressing the play button.
     */
    public void initForPlay() {
        navxMicro.setNavxStatus();
    }

    public void setRobotOrientation(double targetYaw, double motorSpeed) {
        // If the navx is working, then set the robot orientation with navx
        // This will do nothing if the robot is already at the cirrect orientation,
        //  otherwise the robot will be spun a little bit more to accurarely position it.
        if (navxMicro.navxIsWorking()) {
            navxMicro.setRobotOrientation(targetYaw, motorSpeed);
        }
        else {
            double curYaw = encoderNav.getCurrentYaw();
            // Create a NavigationException object to set the terminating conditions
            NavigationException.NavExceptions[] exceptions = {NavigationException.NavExceptions.OPMODE_NOTACTIVE,
                    NavigationException.NavExceptions.TIMED_OUT};
            // Calculate the timeout based on the targetYaw and currentYaw
            // at the rate of 100 milliseconds per degree of rotation at full speed
            long timeoutMillis = (long) Math.abs(this.distanceBetweenAngles(targetYaw, curYaw) * 100 / motorSpeed);
            NavigationException navException = new NavigationException(robot, curOpMode, this, timeoutMillis, exceptions);
            // First, do the encoder based turning.
            encoderNav.setRobotOrientation(targetYaw, motorSpeed, navException);
        }
    }

    public double distanceBetweenAngles(double angle1, double angle2) {
        // Both angle1 and angle2 are assumed to be positive numbers between 0 and 360
        // The returnValue is between 0 and 180.
        double angleDistance= Math.abs(angle1 - angle2);

        if (angleDistance > 180) {
            angleDistance = 360 - angleDistance;
        }

        return (angleDistance);
    }

    /**
     * Calculates whether the robot has to spin clockwise or counter clockwise to go from
     * currentYaw to targetYaw
     * @param curYaw
     * @param targetYaw
     * @return CLOCKWISE, COUNTERCLOCKWISE, NONE
     */
    public SpinDirection getSpinDirection (double curYaw, double targetYaw) {
        SpinDirection direction = SpinDirection.NONE;
        double diffYaw = targetYaw - curYaw;

        double degreesToTurn = diffYaw>180 ? diffYaw-360 : diffYaw<-180 ? diffYaw+360 : diffYaw;

        if (degreesToTurn < 0) {
            direction = SpinDirection.COUNTERCLOCKWISE;
        } else {
            direction = SpinDirection.CLOCKWISE;
        }
        return (direction);
    }

    public double getDegreesToTurn(double curYaw, double targetYaw) {
        double diffYaw = targetYaw - curYaw;
        double degreesToTurn = diffYaw>180 ? diffYaw-360 : diffYaw<-180 ? diffYaw+360 : diffYaw;
        return (degreesToTurn);
    }

    public void goStraightToDistance(double inches, double degrees,
                                     float speed, boolean driveBackwards) {
        // If navx is working, using navx's goStraightPID() method, else use driveSystem's
        // driveToDistance method
        if (navxMicro.navxIsWorking()) {
            robot.driveSystem.resetDistanceTravelled();
            while ((robot.driveSystem.getDistanceTravelledInInches() < inches) &&
                    robot.curOpMode.opModeIsActive()) {
                robot.navigation.navxMicro.MygoStraightPID(driveBackwards, degrees);
            }
            robot.driveSystem.stop();
            robot.driveSystem.resetDistanceTravelled();
        } else {
            // Use purely encoder based navigation
            robot.driveSystem.driveToDistance(speed, inches);
        }
    }

    public void printRangeSensorValue() {
        DbgLog.msg("range sensor distance in cm = %f", rangeSensor.getDistance(DistanceUnit.CM));
    }
}
