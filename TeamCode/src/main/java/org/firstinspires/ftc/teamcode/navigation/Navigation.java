package org.firstinspires.ftc.teamcode.navigation;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.FTCRobot;
import org.firstinspires.ftc.teamcode.attachments.BeaconClaim;
import org.firstinspires.ftc.teamcode.drivesys.DriveSystem;
import org.firstinspires.ftc.teamcode.util.JsonReaders.JsonReader;
import org.firstinspires.ftc.teamcode.util.JsonReaders.NavigationOptionsReader;
import org.firstinspires.ftc.teamcode.util.LoopStatistics;
import org.firstinspires.ftc.teamcode.util.RepetitiveActions;
import org.firstinspires.ftc.teamcode.navigation.MRGyro;
import org.json.JSONObject;



/*
 * Copyright (c) 2016 Robocracy 9773
 */

public class Navigation {
    FTCRobot robot;
    LinearOpMode curOpMode;
    JSONObject navOptObj;
    public MRGyro gyro;
    public LineFollow lf;
    public NavxMicro navxMicro;
    public EncoderNavigation encoderNav;
    public ModernRoboticsI2cRangeSensor rangeSensor;
    public double lfMaxSpeed=1.0, straightDrMaxSpeed=1.0, turnMaxSpeed=1.0;
    public double driveSysTeleopMaxSpeed=1.0;
    private RepetitiveActions.LoopRuntime driveToDistInstr, driveTillWhitelineInstr, turnRobotInstr;
    private RepetitiveActions.LoopRuntime driveTillBeaconInstr;
    private RepetitiveActions.NavxDegrees navxDegreesInstr;
    private RepetitiveActions.RangeSensorDistance rangeInstr;


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
            /*this.navxMicro = new NavxMicro(curOpMode, robot, this, navOption.getIMUDIMname(),
                    navOption.getIMUportNum(), navOption.getIMUVariableDouble("driveSysInitialPower"),
                    navOption.getIMUVariableDouble("angleTolerance"), navOption.getIMUVariableDouble("straightPID_kp"),
                    navOption.getIMUVariableDouble("turnPID_kp"), navOption.getIMUVariableDouble("PID_minSpeed"),
                    navOption.getIMUVariableDouble("PID_maxSpeed"));*/
            this.gyro = new MRGyro(robot,curOpMode);
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

        this.encoderNav = new EncoderNavigation(robot, robot.driveSystem, curOpMode, this);

        // Instantiate the common instrumentation objects (declared as inner classes in RepetitiveActions class
        navxDegreesInstr = robot.repActions.new NavxDegrees(this.navxMicro, true);
        rangeInstr = robot.repActions.new RangeSensorDistance(this.rangeSensor, true);
        driveToDistInstr = robot.repActions.new LoopRuntime(RepetitiveActions.LoopType.DRIVE_TO_DISTANCE);
        driveTillWhitelineInstr = robot.repActions.new LoopRuntime(RepetitiveActions.LoopType.DRIVE_UNTIL_WHITELINE);
        driveTillBeaconInstr = robot.repActions.new LoopRuntime(RepetitiveActions.LoopType.DRIVE_TILL_BEACON);
        turnRobotInstr = robot.repActions.new LoopRuntime(RepetitiveActions.LoopType.TURN_ROBOT);
    }

    public void printRangeSensorValue() {
        DbgLog.msg("ftc9773: range sensor distance in cm = %f", rangeSensor.getDistance(DistanceUnit.CM));
    }

    public void printNavigationValues() {
        DbgLog.msg("ftc9773: encoderYaw=%f, navxYaw=%f", encoderNav.getCurrentYaw(), navxMicro.getModifiedYaw());
        DbgLog.msg("ftc9773: navxPitch=%f, RangeSensor value inches = %f, ods light detected=%f", navxMicro.getPitch(),
                rangeSensor.getDistance(DistanceUnit.INCH), lf.lightSensor.getLightDetected());
        DbgLog.msg("ftc9773: Drive system Encoder values:");
        robot.driveSystem.printCurrentPosition();
    }

    /**
     * Initialize the navigation system just after pressing the play button.
     */
    public void initForPlay() {
        navxMicro.setNavxStatus();
    }

    public void close() {
        DbgLog.msg("ftc9773: Closing all the file objects");
        if (navxDegreesInstr != null) { navxDegreesInstr.closeLog(); }
        if (rangeInstr != null) { rangeInstr.closeLog(); }
        if (driveToDistInstr != null) { driveToDistInstr.closeLog(); }
        if (driveTillWhitelineInstr != null) { driveTillWhitelineInstr.closeLog(); }
        if (turnRobotInstr != null) { turnRobotInstr.closeLog(); }
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

    /**
     * Converts the given targetYaw into the angle to turn.  Returns a value between [-180, +180]
     * @param curYaw  current yaw value
     * @param targetYaw  target yaw value
     *      The targetYaw is with respect to the initial autonomous starting position
     *      The initial orientation of the robot at the beginning of the autonomous period
     *      is '0'. targetYaw is between 0 to 360 degrees.

     * @return degreesToTurn
     */
    public double getDegreesToTurn(double curYaw, double targetYaw) {
        double diffYaw = targetYaw - curYaw;
        double degreesToTurn = diffYaw>180 ? diffYaw-360 : diffYaw<-180 ? diffYaw+360 : diffYaw;
        return (degreesToTurn);
    }

    public double getTargetYaw(double curYaw, double angleToTurn) {
        double sum = curYaw + angleToTurn;
        double targetYaw = (sum>360) ? (sum-360) : ((sum<0) ? (360+sum) : sum);
        return (targetYaw);
    }

    public void untiltRobot(boolean originallyGoingBackward, double degrees, float speed) {
        NavigationChecks navChecks = new NavigationChecks(robot, curOpMode, this);
        // Do not move more than 15 inches; this is the upper limit but not the absolute distance to travel
        NavigationChecks.EncoderCheckForDistance encodercheck = navChecks.new EncoderCheckForDistance(15);
        // Do not move for more than 5 seconds
        NavigationChecks.TimeoutCheck timeoutCheck = navChecks.new TimeoutCheck(5000);
        NavigationChecks.OpmodeInactiveCheck opmodeCheck = navChecks.new OpmodeInactiveCheck();
        navChecks.addNewCheck(encodercheck);
        navChecks.addNewCheck(timeoutCheck);
        navChecks.addNewCheck(opmodeCheck);
        // For un-tilting, the robot has to move in the reverse direction of the original direction.
        boolean driveBackwards = originallyGoingBackward ? false : true;

        robot.repActions.addAction(driveToDistInstr);
        robot.repActions.startActions();
//        LoopStatistics instr = new LoopStatistics();
//        instr.startLoopInstrumentation();
        // move until the robot tilt goes down below 3 degrees
        while (!navChecks.stopNavigation() && (navxMicro.getPitch() > 3)) {
            robot.navigation.navxMicro.navxGoStraightPID(driveBackwards, degrees, speed);
//            instr.updateLoopInstrumentation();
            robot.repActions.repeatActions();
        }
//        instr.printLoopInstrumentation();
        robot.repActions.printToConsole();
        robot.repActions.writeToFile();
        robot.repActions.removeAction(driveToDistInstr);
    }

    public void goStraightToDistance(double inches, double degrees, float speed) {
        // First, disable the color sensor
//        robot.beaconClaimObj.disableColorSensor();
//        DbgLog.msg("Disabled color sensor");
        //rangeSensorState.setEnabled(false);
        // If navx is working, using navx's goStraightPID() method, else use driveSystem's
        // driveToDistance method
        NavigationChecks navChecks = new NavigationChecks(robot, curOpMode, this);
        NavigationChecks.EncoderCheckForDistance encodercheck = navChecks.new EncoderCheckForDistance(inches);
        NavigationChecks.OpmodeInactiveCheck opmodeCheck = navChecks.new OpmodeInactiveCheck();
        navChecks.addNewCheck(opmodeCheck);
        navChecks.addNewCheck(encodercheck);
//        LoopStatistics instr = new LoopStatistics();
        robot.repActions.addAction(rangeInstr);
        robot.repActions.addAction(driveToDistInstr);
        boolean driveBackwards = inches < 0 ? true : false;
        if (navxMicro.navxIsWorking()) {
            DbgLog.msg("ftc9773: Navx is working");
            NavigationChecks.CheckRobotTilting tiltingCheck = navChecks.new CheckRobotTilting(10);
            navChecks.addNewCheck(tiltingCheck);
            robot.repActions.addAction(navxDegreesInstr);
//            instr.startLoopInstrumentation();
            robot.repActions.startActions();
            while (!navChecks.stopNavigation()) {
                robot.navigation.navxMicro.navxGoStraightPID(driveBackwards, degrees, speed);
//                instr.updateLoopInstrumentation();
                robot.repActions.repeatActions();
                if (tiltingCheck.stopNavigation()) {
                    DbgLog.msg("ftc9773: tilting detected");
                    this.untiltRobot(driveBackwards, degrees, speed);
                }
            }
            robot.driveSystem.stop();
            DbgLog.msg("ftc9773: stop navigation condition met");
//            instr.printLoopInstrumentation();
            robot.repActions.printToConsole();
            robot.repActions.writeToFile();
            robot.repActions.removeAction(navxDegreesInstr);
            // Update the encoderNav's current yaw with that of navxMicro
            encoderNav.setCurrentYaw(navxMicro.getModifiedYaw());
        } else {
            DbgLog.msg("ftc9773: Navx is not working");
            // Use purely encoder based navigation
            DbgLog.msg("ftc9773: Speed: %f, distance: %f", speed, inches);
            if (inches < 0) {
                // If driving backwards, then negate the speed
                speed = -speed;
            }
//            instr.startLoopInstrumentation();
            robot.repActions.startActions();
            while (!navChecks.stopNavigation()) {
                robot.driveSystem.drive(speed, 0);
//                instr.updateLoopInstrumentation();
                robot.repActions.repeatActions();
            }
            robot.driveSystem.stop();
//            instr.printLoopInstrumentation();
            robot.repActions.printToConsole();
            robot.repActions.writeToFile();
        }
        robot.repActions.removeAction(rangeInstr);
        robot.repActions.removeAction(driveToDistInstr);
    }

    public void goStraightToWhiteLine(double degrees, float motorSpeed, boolean driveBackwards) {
        NavigationChecks navChecks = new NavigationChecks(robot, curOpMode, this);
        NavigationChecks.CheckForWhiteLine check1 = navChecks.new CheckForWhiteLine();
        NavigationChecks.OpmodeInactiveCheck check2 = navChecks.new OpmodeInactiveCheck();
        navChecks.addNewCheck(check1);
        navChecks.addNewCheck(check2);
//        LoopStatistics instr = new LoopStatistics();
        robot.repActions.addAction(driveTillWhitelineInstr);
        robot.repActions.addAction(navxDegreesInstr);
        // Start beacon color scanning
        //robot.beaconClaimObj.startBeaconScanning();
        if (navxMicro.navxIsWorking()) {
            NavigationChecks.CheckRobotTilting check3 = navChecks.new CheckRobotTilting(10);
            navChecks.addNewCheck(check3);
//            instr.startLoopInstrumentation();
            robot.repActions.startActions();
            while (!navChecks.stopNavigation()) {
                robot.navigation.navxMicro.navxGoStraightPID(driveBackwards, degrees, motorSpeed);
//                instr.updateLoopInstrumentation();
                robot.repActions.repeatActions();
                //robot.beaconClaimObj.updateBeaconScanValues();
            }
            robot.driveSystem.stop();
//            instr.printLoopInstrumentation();
            robot.repActions.printToConsole();
            robot.repActions.writeToFile();
            // Update the encoderNav's current yaw with that of navxMicro
            encoderNav.setCurrentYaw(navxMicro.getModifiedYaw());
        } else {
            // Use purely encoder based navigation
            if (driveBackwards) {
                robot.driveSystem.reverse();
            }
//            instr.startLoopInstrumentation();
            robot.repActions.startActions();
            while (!navChecks.stopNavigation()) {
                robot.driveSystem.drive(motorSpeed, 0);
//                instr.updateLoopInstrumentation();
                robot.repActions.repeatActions();
                //robot.beaconClaimObj.updateBeaconScanValues();
            }
            robot.driveSystem.stop();
//            instr.printLoopInstrumentation();
            robot.repActions.printToConsole();
            robot.repActions.writeToFile();
            if (driveBackwards) {
                robot.driveSystem.reverse();
            }
        }
        robot.repActions.removeAction(driveTillWhitelineInstr);
        robot.repActions.removeAction(navxDegreesInstr);
        // Print the first and second scanned values for information purpose only
        //robot.beaconClaimObj.printBeaconScanningData();
        //robot.beaconClaimObj.stopBeaconScanning();
    }

    public void driveUntilAllianceBeacon(double motorSpeed, double degrees,
                                         double distance1, double distance2) {
        // Determine the first and second colors
        double distance=0.0;
        boolean driveBackwards=false;
        BeaconClaim.BeaconColor color = robot.beaconClaimObj.getBeaconColor();
        if (color == BeaconClaim.BeaconColor.RED) {
            if (robot.autonomousActions.allianceColor.equalsIgnoreCase("red")) {
                distance = distance2;
            } else if (robot.autonomousActions.allianceColor.equalsIgnoreCase("blue")) {
                distance = distance1;
            }
        } else if (color == BeaconClaim.BeaconColor.BLUE) {
            if (robot.autonomousActions.allianceColor.equalsIgnoreCase("red")) {
                distance = distance1;
            } else if (robot.autonomousActions.allianceColor.equalsIgnoreCase("blue")) {
                distance = distance2;
            }
        } else { // If no color is detected don't waste time and move on to the next step
            return;
        }
//        LoopStatistics instr = new LoopStatistics();
        robot.repActions.addAction(driveTillBeaconInstr);
        robot.repActions.addAction(navxDegreesInstr);
        robot.repActions.addAction(rangeInstr);
        NavigationChecks navChecks = new NavigationChecks(robot, curOpMode, this);
        NavigationChecks.OpmodeInactiveCheck opmodeCheck = navChecks.new OpmodeInactiveCheck();
        navChecks.addNewCheck(opmodeCheck);
        driveBackwards = distance < 0 ? true : false;
        NavigationChecks.EncoderCheckForDistance distanceCheck = navChecks.new EncoderCheckForDistance(distance);
        navChecks.addNewCheck(distanceCheck);
        if (navxMicro.navxIsWorking()) {
//            instr.startLoopInstrumentation();
            robot.repActions.startActions();
            while (!navChecks.stopNavigation()) {
                robot.navigation.navxMicro.navxGoStraightPID(driveBackwards, degrees, (float) motorSpeed);
//                instr.updateLoopInstrumentation();
                robot.repActions.repeatActions();
            }
//            instr.printLoopInstrumentation();
            robot.repActions.printToConsole();
            robot.repActions.writeToFile();
            robot.driveSystem.stop();
            // Update the encoderNav's current yaw with that of navxMicro
            encoderNav.setCurrentYaw(navxMicro.getModifiedYaw());
        } else {
            robot.driveSystem.driveToDistance((float)motorSpeed, distance);
        }
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
        NavigationChecks.OpmodeInactiveCheck check2 = navigationChecks.new OpmodeInactiveCheck();
        navigationChecks.addNewCheck(check2);

        robot.repActions.addAction(navxDegreesInstr);
        robot.repActions.addAction(turnRobotInstr);

        DriveSystem.ElapsedEncoderCounts elapsedEncoderCounts = robot.driveSystem.getNewElapsedCountsObj();
        elapsedEncoderCounts.reset();

        // If the navx is working, then set the robot orientation with navx
        if (navxMicro.navxIsWorking()) {
            curOpMode.telemetry.addData("Set Robot Orientation", "Using Navx");
            curOpMode.telemetry.update();
            DbgLog.msg("ftc9773: Set Robot Orientation, Using Navx");
            // The difference between the encoder-based degrees and navx based degrees can easily
            // go upto 10 degrees even when navx is working well.  So, we should not have too low
            // value for the CheckWhileTurning constructor.  Ensure that we do not check for less
            // 30 degree deviation between the encoder-based and navx-based angles.
//            double degreesToCheck = Math.max(this.distanceBetweenAngles(targetYaw, curYaw) /2, 30);
//            NavigationChecks.CheckNavxWhileTurning check3 = navigationChecks.new
//                    CheckNavxWhileTurning(degreesToCheck);
//            navigationChecks.addNewCheck(check3);
            NavigationChecks.CheckNavxIsWorking navxCheck = navigationChecks.new CheckNavxIsWorking();
            navigationChecks.addNewCheck(navxCheck);
            navxMicro.setRobotOrientation(targetYaw, motorSpeed, navigationChecks);
            if ((navigationChecks.stopNavCriterion != null) &&
                    ((navigationChecks.stopNavCriterion.navcheck == NavigationChecks.NavChecksSupported.CROSSCHECK_NAVX_WITH_ENCODERS)
                    || (navigationChecks.stopNavCriterion.navcheck == NavigationChecks.NavChecksSupported.CHECK_NAVX_IS_WORKING)))
            {
                double encoder_degreesTurned = elapsedEncoderCounts.getDegreesTurned();
                encoderNav.updateCurrentYaw(encoder_degreesTurned);
                elapsedEncoderCounts.reset();
                curOpMode.telemetry.addData("Set Robot Orientation", "Not Using Navx");
                curOpMode.telemetry.update();
                DbgLog.msg("ftc9773: Set Robot Orientation, Not Using Navx");
//                navigationChecks.removeCheck(check3);
                navigationChecks.removeCheck(navxCheck);
                encoderNav.setRobotOrientation(targetYaw, motorSpeed, navigationChecks);
                encoder_degreesTurned = elapsedEncoderCounts.getDegreesTurned();
                encoderNav.updateCurrentYaw(encoder_degreesTurned);
                // Set the navx status again
                navxMicro.setNavxStatus();
            } else if (navigationChecks.stopNavCriterion == null) {
                // navx worked without any problems; Set the encoderNav's currentYaw to the navx yaw value
                encoderNav.setCurrentYaw(navxMicro.getModifiedYaw());
            }
        }
        else {
            // First, do the encoder based turning.
            curOpMode.telemetry.addData("Set Robot Orientation", "Not Using Navx");
            curOpMode.telemetry.update();
            DbgLog.msg("ftc9773: Set Robot Orientation, Not Using Navx");
            encoderNav.setRobotOrientation(targetYaw, motorSpeed, navigationChecks);
            encoderNav.updateCurrentYaw(elapsedEncoderCounts.getDegreesTurned());
            DbgLog.msg("ftc9773: currYaw: %f", encoderNav.getCurrentYaw());
        }
        robot.repActions.removeAction(navxDegreesInstr);
        robot.repActions.removeAction(turnRobotInstr);
    }

    /**
     *
     * @param shiftDistance +ve => shift right, -ve => shift left
     * @param moveDistance  +ve => move forward; -ve => move backward
     * @param speed
     * @param returnToSamePos
     */
    public void shiftRobot(double shiftDistance, double moveDistance, double speed,
                           boolean returnToSamePos){
        boolean isForward = (moveDistance >= 0) ? true : false;
        double diagonal = Math.sqrt(Math.pow(moveDistance, 2) + Math.pow(shiftDistance, 2));
        double angle = 90 - Math.toDegrees(Math.asin(Math.abs(moveDistance/diagonal)));

        // diagonal should have the same sign as the moveDistance
        diagonal *= Math.signum(moveDistance);
        // If moveDistance and shiftDistance have opposite signs -- i.e. move forward & shift left
        //  or move backward & shift right -- then turn counter clockwise, else turn clockwise
        angle *= Math.signum(moveDistance) * Math.signum(shiftDistance);
        DbgLog.msg("ftc9773: shiftDistance=%f, diagonal=%f, moveDistance=%f, isForward=%b, speed=%f, angle=%f",
                shiftDistance, diagonal, moveDistance, isForward, speed, angle);
//        if (isForward) {
//            if (shiftDistance < 0) {
//                angle *= -1;
//            }
//        } else{
//            if (shiftDistance > 0){
//                angle *= -1;
//            }
//            diagonal = -diagonal;
//        }

        // Step 1.  Turn the robot to move forward / backward
        double startingYaw = (navxMicro.navxIsWorking() ? navxMicro.getModifiedYaw() : encoderNav.getCurrentYaw());
        double turningYaw = this.getTargetYaw(startingYaw, angle);
        DbgLog.msg("ftc9773: startingYaw=%f, turningYaw=%f", startingYaw, turningYaw);
        this.setRobotOrientation(turningYaw, speed);
        // Step 2:  Drive to diagonal distance
        this.goStraightToDistance(diagonal, turningYaw, (float)speed);
        //Step 3: Turn to the original orientation again
        this.setRobotOrientation(startingYaw, speed);
        // Step 4:  return to the same position is specified
        if (returnToSamePos) {
            this.goStraightToDistance(moveDistance, startingYaw, (float)speed);
        }
    }
}
