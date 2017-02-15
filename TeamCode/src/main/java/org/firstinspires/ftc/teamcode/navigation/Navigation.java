package org.firstinspires.ftc.teamcode.navigation;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.FTCRobot;
import org.firstinspires.ftc.teamcode.attachments.BeaconClaim;
import org.firstinspires.ftc.teamcode.drivesys.DriveSystem;
import org.firstinspires.ftc.teamcode.util.BackgroundTasks;
import org.firstinspires.ftc.teamcode.util.Instrumentation;
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
    public double lfMaxSpeed=1.0, straightDrMaxSpeed=1.0, turnMaxSpeed=1.0;
    public double driveSysTeleopMaxSpeed=1.0;
    private Instrumentation.LoopRuntime driveToDistInstr, driveTillWhitelineInstr, turnRobotInstr;
    private Instrumentation.LoopRuntime driveTillBeaconInstr;
    private Instrumentation.NavxDegrees navxDegreesInstr;
    private Instrumentation.RangeSensorDistance rangeInstr;


    public enum SpinDirection {CLOCKWISE, COUNTERCLOCKWISE, NONE}

    public Navigation(FTCRobot robot, LinearOpMode curOpMode, String navOptionStr) {
        this.robot = robot;
        this.curOpMode = curOpMode;
        double rangeSensorRunningAvg=0.0;

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
            if (navOption.getIMUType().equalsIgnoreCase("navx-micro")) {
                this.navxMicro = new NavxMicro(curOpMode, robot, this, navOption.getIMUDIMname(),
                        navOption.getIMUportNum(), navOption.getIMUVariableDouble("driveSysInitialPower"),
                        navOption.getIMUVariableDouble("angleTolerance"), navOption.getIMUVariableDouble("straightPID_kp"),
                        navOption.getIMUVariableDouble("turnPID_kp"), navOption.getIMUVariableDouble("PID_minSpeed"),
                        navOption.getIMUVariableDouble("PID_maxSpeed"));
            } else if (navOption.getIMUType().equalsIgnoreCase("MRgyro")) {
                // ToDo for Luke:  instantiate MR gyro object here
                DbgLog.msg("ftc9773: instantiating MR gyro");
                curOpMode.telemetry.addData("ftc9773:", "instantiating MR gyro");
                curOpMode.telemetry.update();
            }
        }
        else {
            this.navxMicro = null;
        }

        if (navOption.rangeSensorExists()) {
            this.rangeSensor = curOpMode.hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeSensor1");
            rangeSensorRunningAvg = navOption.getRangeSensorRunningAvgWeight();
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

        // Instantiate the common instrumentation objects (declared as inner classes in Instrumentation class
        navxDegreesInstr = robot.instrumentation.new NavxDegrees(this.navxMicro, true);
        rangeInstr = robot.instrumentation.new RangeSensorDistance(this.rangeSensor, rangeSensorRunningAvg, true);
        driveToDistInstr = robot.instrumentation.new LoopRuntime(Instrumentation.LoopType.DRIVE_TO_DISTANCE);
        driveTillWhitelineInstr = robot.instrumentation.new LoopRuntime(Instrumentation.LoopType.DRIVE_UNTIL_WHITELINE);
        driveTillBeaconInstr = robot.instrumentation.new LoopRuntime(Instrumentation.LoopType.DRIVE_TILL_BEACON);
        turnRobotInstr = robot.instrumentation.new LoopRuntime(Instrumentation.LoopType.TURN_ROBOT);
    }

    public void printRangeSensorValue() {
        DbgLog.msg("ftc9773: range sensor distance in cm = %f", rangeSensor.getDistance(DistanceUnit.CM));
    }

    public void printNavigationValues() {
        DbgLog.msg("ftc9773: encoderYaw=%f, navxYaw=%f", encoderNav.getCurrentYaw(), navxMicro.getModifiedYaw());
        DbgLog.msg("ftc9773: navxPitch=%f, RangeSensor value cm = %f, ods light detected=%f", navxMicro.getPitch(),
                rangeSensor.getDistance(DistanceUnit.CM), lf.lightSensor.getLightDetected());
        DbgLog.msg("ftc9773: Drive system Encoder values:");
        robot.driveSystem.printCurrentPosition();
    }

    /**
     * Initialize the navigation system just after pressing the play button.
     */
    public void initForPlay() {
        if (navxMicro != null){
            navxMicro.setNavxStatus();
        }
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

        robot.instrumentation.addAction(driveToDistInstr);
        robot.instrumentation.reset();
//        LoopStatistics instr = new LoopStatistics();
//        instr.startLoopInstrumentation();
        // move until the robot tilt goes down below 3 degrees
        while (!navChecks.stopNavigation() && (navxMicro.getPitch() > 3)) {
            robot.navigation.navxMicro.navxGoStraightPID(driveBackwards, degrees, speed);
//            instr.updateLoopInstrumentation();
            robot.instrumentation.addInstrData();
        }
//        instr.printLoopInstrumentation();
        robot.instrumentation.printToConsole();
        robot.instrumentation.writeToFile();
        robot.instrumentation.removeAction(driveToDistInstr);
    }

    public void goStraightTillNavxIsStable(double inches, double degrees, double degreeTolerance,
                                           float speed, int numUpdatesToSettle) {
        // Before measuring the distance using range sensor, we should wait until the robot settles
        // on a path roughly parallel to the wall.  This can be done by waiting until numUpdatesToSettle
        // number of consecutive navx readings have been within the tolerance of degreeTolerance.
        if (navxMicro.navxIsWorking()) {
            NavigationChecks navChecks = new NavigationChecks(robot, curOpMode, this);
            NavigationChecks.EncoderCheckForDistance encodercheck = navChecks.new EncoderCheckForDistance(inches);
            NavigationChecks.OpmodeInactiveCheck opmodeCheck = navChecks.new OpmodeInactiveCheck();
            navChecks.addNewCheck(opmodeCheck);
            navChecks.addNewCheck(encodercheck);
            robot.instrumentation.addAction(rangeInstr);
            robot.instrumentation.addAction(driveToDistInstr);
            Instrumentation.NavxYawMonitor yawMonitor = robot.instrumentation.new NavxYawMonitor(this, navxMicro,
                    degrees, degreeTolerance, numUpdatesToSettle);
            robot.instrumentation.addAction(yawMonitor);

            boolean driveBackwards = inches < 0 ? true : false;

            DbgLog.msg("ftc9773: Navx is working");
            NavigationChecks.CheckRobotTilting tiltingCheck = navChecks.new CheckRobotTilting(10);
            navChecks.addNewCheck(tiltingCheck);
            robot.instrumentation.addAction(navxDegreesInstr);
            robot.instrumentation.reset();
            while (!navChecks.stopNavigation()) {
                robot.navigation.navxMicro.navxGoStraightPID(driveBackwards, degrees, speed);
                robot.instrumentation.addInstrData();
                if (yawMonitor.targetYawReachedAndStable) {
                    // remove the yaw monitor,
                    // reset rangeSensorDtistance repeat action,
                    // get running average over 100 milli seconds,
                    // determine the spin angle based on the distance from wall,
                    // spin the robot and continue the while loop
                    yawMonitor.printToConsole();
                    robot.instrumentation.removeAction(yawMonitor);
                    break;
                }
                if (tiltingCheck.stopNavigation()) {
                    DbgLog.msg("ftc9773: tilting detected");
                    this.untiltRobot(driveBackwards, degrees, speed);
                }
            }
            robot.driveSystem.stop();
            robot.instrumentation.printToConsole();
            robot.instrumentation.writeToFile();
            robot.instrumentation.removeAction(navxDegreesInstr);
            robot.instrumentation.removeAction(rangeInstr);
            robot.instrumentation.removeAction(driveToDistInstr);
        } else {
            // call regular goStraightToDistance
            goStraightToDistance(inches, degrees, speed);
        }
    }

    public void goStraightToDistance(double inches, double degrees, float speed) {
        // If navx is working, using navx's goStraightPID() method, else use driveSystem's
        // driveToDistance method
        NavigationChecks navChecks = new NavigationChecks(robot, curOpMode, this);
        NavigationChecks.EncoderCheckForDistance encodercheck = navChecks.new EncoderCheckForDistance(inches);
        NavigationChecks.OpmodeInactiveCheck opmodeCheck = navChecks.new OpmodeInactiveCheck();
        navChecks.addNewCheck(opmodeCheck);
        navChecks.addNewCheck(encodercheck);
        robot.instrumentation.addAction(rangeInstr);
        robot.instrumentation.addAction(driveToDistInstr);
        boolean driveBackwards = inches < 0 ? true : false;
        if (navxMicro.navxIsWorking()) {
            DbgLog.msg("ftc9773: Navx is working");
            NavigationChecks.CheckRobotTilting tiltingCheck = navChecks.new CheckRobotTilting(10);
            navChecks.addNewCheck(tiltingCheck);
            robot.instrumentation.addAction(navxDegreesInstr);
            robot.instrumentation.reset();
            while (!navChecks.stopNavigation()) {
                robot.navigation.navxMicro.navxGoStraightPID(driveBackwards, degrees, speed);
                robot.instrumentation.addInstrData();
                if (tiltingCheck.stopNavigation()) {
                    DbgLog.msg("ftc9773: tilting detected");
                    this.untiltRobot(driveBackwards, degrees, speed);
                }
            }
            robot.driveSystem.stop();
            robot.instrumentation.printToConsole();
            robot.instrumentation.writeToFile();
            robot.instrumentation.removeAction(navxDegreesInstr);
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
            robot.instrumentation.reset();
            while (!navChecks.stopNavigation()) {
                robot.driveSystem.drive(speed, 0);
//                instr.updateLoopInstrumentation();
                robot.instrumentation.addInstrData();
            }
            robot.driveSystem.stop();
//            instr.printLoopInstrumentation();
            robot.instrumentation.printToConsole();
            robot.instrumentation.writeToFile();
        }
        robot.instrumentation.removeAction(rangeInstr);
        robot.instrumentation.removeAction(driveToDistInstr);
    }

    public void goStraightToWhiteLine(double degrees, float motorSpeed, boolean driveBackwards) {
        NavigationChecks navChecks = new NavigationChecks(robot, curOpMode, this);
        NavigationChecks.CheckForWhiteLine check1 = navChecks.new CheckForWhiteLine();
        NavigationChecks.OpmodeInactiveCheck check2 = navChecks.new OpmodeInactiveCheck();
        navChecks.addNewCheck(check1);
        navChecks.addNewCheck(check2);
        robot.instrumentation.addAction(driveTillWhitelineInstr);
        robot.instrumentation.addAction(navxDegreesInstr);
        // Determine the distance from wall and see if beaconServo needs to be retracted
        // It might have been pre-extended before.
        double distFromWall = rangeSensor.getDistance(DistanceUnit.CM);
        BackgroundTasks.BeaconServoExtender beaconServoExtender =
                robot.backgroundTasks.new BeaconServoExtender(robot.beaconClaimObj, 800);
        beaconServoExtender.setTaskParams((distFromWall-6), 800);
        beaconServoExtender.startTask();

        if (navxMicro.navxIsWorking()) {
            NavigationChecks.CheckRobotTilting check3 = navChecks.new CheckRobotTilting(10);
            navChecks.addNewCheck(check3);
            robot.instrumentation.reset();
            while (!navChecks.stopNavigation()) {
                robot.navigation.navxMicro.navxGoStraightPID(driveBackwards, degrees, motorSpeed);
                robot.instrumentation.addInstrData();
                beaconServoExtender.continueTask();
            }
            robot.driveSystem.stop();
            robot.instrumentation.printToConsole();
            robot.instrumentation.writeToFile();
            // Update the encoderNav's current yaw with that of navxMicro
            encoderNav.setCurrentYaw(navxMicro.getModifiedYaw());
        } else {
            // Use purely encoder based navigation
            if (driveBackwards) {
                robot.driveSystem.reverse();
            }
            robot.instrumentation.reset();
            while (!navChecks.stopNavigation()) {
                robot.driveSystem.drive(motorSpeed, 0);
                robot.instrumentation.addInstrData();
                beaconServoExtender.continueTask();
            }
            robot.driveSystem.stop();
            robot.instrumentation.printToConsole();
            robot.instrumentation.writeToFile();
            if (driveBackwards) {
                robot.driveSystem.reverse();
            }
        }
        beaconServoExtender.endTask();
        robot.instrumentation.removeAction(driveTillWhitelineInstr);
        robot.instrumentation.removeAction(navxDegreesInstr);
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
        robot.instrumentation.addAction(driveTillBeaconInstr);
        robot.instrumentation.addAction(navxDegreesInstr);
        robot.instrumentation.addAction(rangeInstr);
        NavigationChecks navChecks = new NavigationChecks(robot, curOpMode, this);
        NavigationChecks.OpmodeInactiveCheck opmodeCheck = navChecks.new OpmodeInactiveCheck();
        navChecks.addNewCheck(opmodeCheck);
        driveBackwards = distance < 0 ? true : false;
        NavigationChecks.EncoderCheckForDistance distanceCheck = navChecks.new EncoderCheckForDistance(distance);
        navChecks.addNewCheck(distanceCheck);
        // Determine the distance from wall and see if beaconServo needs to be pre-extended
        double distFromWall = rangeSensor.getDistance(DistanceUnit.CM);
        BackgroundTasks.BeaconServoExtender beaconServoExtender =
                robot.backgroundTasks.new BeaconServoExtender(robot.beaconClaimObj, 800);
        beaconServoExtender.setTaskParams((distFromWall-4), 800);
        beaconServoExtender.startTask();
        if (navxMicro.navxIsWorking()) {
            robot.instrumentation.reset();
            while (!navChecks.stopNavigation()) {
                robot.navigation.navxMicro.navxGoStraightPID(driveBackwards, degrees, (float) motorSpeed);
                robot.instrumentation.addInstrData();
                beaconServoExtender.continueTask();
            }
            robot.instrumentation.printToConsole();
            robot.instrumentation.writeToFile();
            robot.driveSystem.stop();
            // Update the encoderNav's current yaw with that of navxMicro
            encoderNav.setCurrentYaw(navxMicro.getModifiedYaw());
        } else {
            robot.driveSystem.driveToDistance((float)motorSpeed, distance);
        }
        beaconServoExtender.endTask();
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

        robot.instrumentation.addAction(navxDegreesInstr);
        robot.instrumentation.addAction(turnRobotInstr);

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
//            NavigationChecks.CrossCheckNavxWhileTurning check3 = navigationChecks.new
//                    CrossCheckNavxWhileTurning(degreesToCheck);
//            navigationChecks.addNewCheck(check3);
            NavigationChecks.CheckNavxIsWorking navxCheck = navigationChecks.new CheckNavxIsWorking();
            navigationChecks.addNewCheck(navxCheck);
            NavigationChecks.CheckNavxTargetYawReached targetYawCheck = navigationChecks.new CheckNavxTargetYawReached(targetYaw);
            navigationChecks.addNewCheck(targetYawCheck);
            SpinDirection cwccw = getSpinDirection(navxMicro.getModifiedYaw(), targetYaw);
            double leftPower=0.0, rightPower=0.0;
            if (cwccw == SpinDirection.CLOCKWISE) {
                leftPower = motorSpeed;
                rightPower = -motorSpeed;
            }
            if (cwccw == SpinDirection.COUNTERCLOCKWISE) {
                leftPower = -motorSpeed;
                rightPower = motorSpeed;
            }
            DbgLog.msg("ftc9773: power left = %f, right = %f",leftPower, rightPower);
            robot.instrumentation.reset();
            while (!navigationChecks.stopNavigation()) {
                robot.driveSystem.turnOrSpin(leftPower, rightPower);
                robot.instrumentation.addInstrData();
            }
            this.robot.driveSystem.stop();
            robot.instrumentation.printToConsole();
//            navxMicro.setRobotOrientation(targetYaw, motorSpeed, navigationChecks);
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
            } else if (navigationChecks.stopNavCriterion.navcheck == NavigationChecks.NavChecksSupported.CHECK_TARGET_YAW_NAVX) {
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
        robot.instrumentation.removeAction(navxDegreesInstr);
        robot.instrumentation.removeAction(turnRobotInstr);
    }

    /**
     *
     * @param shiftDistance +ve => shift right, -ve => shift left
     * @param moveDistance  +ve => move forward; -ve => move backward
     * @param speed
     * @param returnToSamePos
     */
    public void shiftRobot(double shiftDistance, double moveDistance, double speed,
                           boolean returnToSamePos, double startingYaw, double endingYaw){
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

        // Step 1.  Turn the robot to move forward / backward
        if (startingYaw < 0)
            startingYaw = (navxMicro.navxIsWorking() ? navxMicro.getModifiedYaw() : encoderNav.getCurrentYaw());
        double turningYaw = this.getTargetYaw(startingYaw, angle);
        DbgLog.msg("ftc9773: startingYaw=%f, turningYaw=%f", startingYaw, turningYaw);
        this.setRobotOrientation(turningYaw, turnMaxSpeed);
        // Step 2:  Drive to diagonal distance
        this.goStraightToDistance(diagonal, turningYaw, (float)speed);
        //Step 3: Turn to the original orientation again
        if (endingYaw >= 0 && endingYaw <= 360)
            this.setRobotOrientation(endingYaw, turnMaxSpeed);
        else
            this.setRobotOrientation(startingYaw, turnMaxSpeed);
        // Step 4:  return to the same position is specified
        if (returnToSamePos) {
            this.goStraightToDistance(moveDistance, startingYaw, (float)speed);
        }
    }
}
