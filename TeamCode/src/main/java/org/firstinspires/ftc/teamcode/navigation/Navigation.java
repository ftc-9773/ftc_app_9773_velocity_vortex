package org.firstinspires.ftc.teamcode.navigation;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.FTCRobot;
import org.firstinspires.ftc.teamcode.drivesys.DriveSystem;
import org.firstinspires.ftc.teamcode.util.FTCi2cDeviceState;
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
    public FTCi2cDeviceState rangeSensorState;

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
            //this.rangeSensorState = new FTCi2cDeviceState((I2cDeviceSynch)rangeSensor);
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

        this.encoderNav = new EncoderNavigation(robot, robot.driveSystem, curOpMode, this);
    }

    /**
     * Initialize the navigation system just after pressing the play button.
     */
    public void initForPlay() {
        navxMicro.setNavxStatus();
    }

    public void setRobotOrientation(double targetYaw, double motorSpeed) {
        // Create a NavigationChecks object to set the terminating conditions
        NavigationChecks navigationChecks = new NavigationChecks(robot, curOpMode, this);
        // Calculate the timeout based on the targetYaw and currentYaw
        // at the rate of 100 milliseconds per degree of rotation at full speed
        double curYaw = encoderNav.getCurrentYaw();
        long timeoutMillis = (long) Math.abs(this.distanceBetweenAngles(targetYaw, curYaw) * 100 / motorSpeed);
        NavigationChecks.TimeoutCheck check1 = navigationChecks.new TimeoutCheck(timeoutMillis);
        navigationChecks.addNewCheck(check1);

        DriveSystem.ElapsedEncoderCounts elapsedEncoderCounts = robot.driveSystem.getNewElapsedCountsObj();
        elapsedEncoderCounts.reset();

        // If the navx is working, then set the robot orientation with navx
        if (navxMicro.navxIsWorking()) {
            curOpMode.telemetry.addData("Set Robot Orientation", "Using Navx");
            curOpMode.telemetry.update();
            NavigationChecks.CheckNavxWhileTurning check2 = navigationChecks.new
                    CheckNavxWhileTurning(this.distanceBetweenAngles(targetYaw, curYaw) /2);
            navigationChecks.addNewCheck(check2);
            navxMicro.setRobotOrientation(targetYaw, motorSpeed, navigationChecks);
            if ((navigationChecks.stopNavCriterion != null) &&
                    (navigationChecks.stopNavCriterion.navcheck == NavigationChecks.NavChecksSupported.CROSSCHECK_NAVX_WITH_ENCODERS)){
                double encoder_degreesTurned = elapsedEncoderCounts.getDegreesTurned();
                encoderNav.updateCurrentYaw(encoder_degreesTurned);
                curOpMode.telemetry.addData("Set Robot Orientation", "Not Using Navx");
                curOpMode.telemetry.update();
                encoderNav.setRobotOrientation(targetYaw, motorSpeed, navigationChecks);
                elapsedEncoderCounts.reset();
            }
            double encoder_degreesTurned = elapsedEncoderCounts.getDegreesTurned();
            encoderNav.updateCurrentYaw(encoder_degreesTurned);
        }
        else {
            // First, do the encoder based turning.
            curOpMode.telemetry.addData("Set Robot Orientation", "Not Using Navx");
            curOpMode.telemetry.update();
            encoderNav.setRobotOrientation(targetYaw, motorSpeed, navigationChecks);
            encoderNav.updateCurrentYaw(elapsedEncoderCounts.getDegreesTurned());
            DbgLog.msg("currYaw: %f", encoderNav.getCurrentYaw());
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
        // First, disable the color sensor
        robot.beaconClaimObj.disableColorSensor();
        DbgLog.msg("Disabled color sensor");
        //rangeSensorState.setEnabled(false);
        // If navx is working, using navx's goStraightPID() method, else use driveSystem's
        // driveToDistance method
        NavigationChecks navChecks = new NavigationChecks(robot, curOpMode, this);
        NavigationChecks.EncoderCheckForDistance check1 = navChecks.new EncoderCheckForDistance(inches);
        NavigationChecks.OpmodeInactiveCheck check2 = navChecks.new OpmodeInactiveCheck();
        navChecks.addNewCheck(check1);
        navChecks.addNewCheck(check2);
        if (navxMicro.navxIsWorking()) {
            while (!navChecks.stopNavigation() && curOpMode.opModeIsActive()) {
                robot.navigation.navxMicro.MygoStraightPID(driveBackwards, degrees);
            }
            robot.driveSystem.stop();
        } else {
            if (driveBackwards) {
                robot.driveSystem.reverse();
            }
            // Use purely encoder based navigation
            DbgLog.msg("Speed: %f, distance: %f", speed, inches);
            robot.driveSystem.driveToDistance(speed, inches);
            if (driveBackwards) {
                robot.driveSystem.reverse();
            }
        }
    }

    public void goStraightToWhiteLine(double degrees, float motorSpeed, boolean driveBackwards) {
        NavigationChecks navChecks = new NavigationChecks(robot, curOpMode, this);
        NavigationChecks.CheckForWhiteLine check1 = navChecks.new CheckForWhiteLine();
        NavigationChecks.OpmodeInactiveCheck check2 = navChecks.new OpmodeInactiveCheck();
        navChecks.addNewCheck(check1);
        navChecks.addNewCheck(check2);
        if (navxMicro.navxIsWorking()) {
            while (!navChecks.stopNavigation() && curOpMode.opModeIsActive()) {
                robot.navigation.navxMicro.MygoStraightPID(driveBackwards, degrees);
            }
            robot.driveSystem.stop();
        } else {
            // Use purely encoder based navigation
            if (driveBackwards) {
                robot.driveSystem.reverse();
            }
            while (!navChecks.stopNavigation() && curOpMode.opModeIsActive()) {
                robot.driveSystem.drive(motorSpeed, 0);
            }
            if (driveBackwards) {
                robot.driveSystem.reverse();
            }
        }
    }

    public void printRangeSensorValue() {
        DbgLog.msg("range sensor distance in cm = %f", rangeSensor.getDistance(DistanceUnit.CM));
    }

    public void shiftRobot(double distance, boolean isForward){
        NavigationChecks navigationChecks = new NavigationChecks(robot, curOpMode, this);
        NavigationChecks.TimeoutCheck timeoutCheck = navigationChecks.new TimeoutCheck(10000);
        NavigationChecks.CheckNavxWhileTurning checkNavxWhileTurning = navigationChecks.new CheckNavxWhileTurning(10);
        NavigationChecks.OpmodeInactiveCheck opmodeCheck = navigationChecks.new OpmodeInactiveCheck();

        navigationChecks.addNewCheck(timeoutCheck);
        navigationChecks.addNewCheck(checkNavxWhileTurning);
        navigationChecks.addNewCheck(opmodeCheck);

        DriveSystem.ElapsedEncoderCounts elapsedEncoderCounts = robot.driveSystem.getNewElapsedCountsObj();
        elapsedEncoderCounts.reset();

        if (navxMicro.navxIsWorking()) {
            navxMicro.shiftRobot(distance, isForward, navigationChecks);
            if (navigationChecks.stopNavCriterion.navcheck == NavigationChecks.NavChecksSupported.CROSSCHECK_NAVX_WITH_ENCODERS){
                double encoder_degreesTurned = elapsedEncoderCounts.getDegreesTurned();
                encoderNav.updateCurrentYaw(encoder_degreesTurned);
                elapsedEncoderCounts.reset();
            }
            double encoder_degreesTurned = elapsedEncoderCounts.getDegreesTurned();
            encoderNav.updateCurrentYaw(encoder_degreesTurned);
        }
        else {
            double curYaw = encoderNav.getCurrentYaw();
            // Create a NavigationChecks object to set the terminating conditions
            NavigationChecks.NavChecksSupported[] exceptions = {NavigationChecks.NavChecksSupported.CHECK_OPMODE_INACTIVE,
                    NavigationChecks.NavChecksSupported.CHECK_TIMEOUT};
            // Calculate the timeout based on the targetYaw and currentYaw
            // at the rate of 100 milliseconds per degree of rotation at full speed
            // First, do the encoder based turning.
            encoderNav.updateCurrentYaw(elapsedEncoderCounts.getDegreesTurned());
        }
    }
}
