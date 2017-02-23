package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.attachments.BeaconClaim;
import org.firstinspires.ftc.teamcode.drivesys.DriveSystem;
import org.firstinspires.ftc.teamcode.util.FileRW;
import org.firstinspires.ftc.teamcode.util.JsonReaders.AutonomousOptionsReader;
import org.firstinspires.ftc.teamcode.util.JsonReaders.JsonReader;
import org.json.JSONException;
import org.json.JSONObject;

import java.util.concurrent.TimeUnit;

/*
 * Copyright (c) 2016 Robocracy 9773
 */

/**
 *
 */
public class AutonomousActions {
    FTCRobot robot;
    LinearOpMode curOpMode;
    public String allianceColor;
    AutonomousOptionsReader autoCfg;
    String replayFilesDir;
    DriveSystem driveSystem;
    final double CM2INCHES = 0.3937;


    public AutonomousActions(FTCRobot robot, LinearOpMode curOpMode, String autoOption,
                             String allianceColor) {
        this.robot = robot;
        this.curOpMode = curOpMode;
        this.driveSystem = robot.driveSystem;
        this.allianceColor = allianceColor;
        autoCfg = new AutonomousOptionsReader(JsonReader.autonomousOptFile, autoOption);
        if (allianceColor.equalsIgnoreCase("Blue"))
            this.replayFilesDir = JsonReader.autonomousBlueDir;
        else if (allianceColor.equalsIgnoreCase("Red"))
            this.replayFilesDir = JsonReader.autonomousRedDir;
    }

    public void replayFileAction(String replayFile) throws InterruptedException {
        FileRW fileRW;
        fileRW = new FileRW(replayFile, false);
        fileRW.getNextLine(); // skip the header row
        String line;
        long startingTime = System.nanoTime();
        long elapsedTime = 0;
        long sleepTime = 0;
        int turnCounter = 0;
        while (((line = fileRW.getNextLine()) != null) && curOpMode.opModeIsActive()) {
            String[] lineElements = line.split(",");
//            DbgLog.msg("ftc9773: lineElements length = %d", lineElements.length);
            if(lineElements.length < 3){
                continue;
            }
            else {
                long timestamp = Long.parseLong(lineElements[0]);
                double speed = Double.parseDouble(lineElements[1]);
                double direction = Double.parseDouble(lineElements[2]);
                elapsedTime = System.nanoTime() - startingTime;
                if (elapsedTime < timestamp) {
                    sleepTime = timestamp - elapsedTime;
                    TimeUnit.NANOSECONDS.sleep(sleepTime);
                }
                if(lineElements.length > 3){
                    if(turnCounter == 0) {
                        DbgLog.msg("ftc9773: Yaw: %f, Target yaw = %s", robot.navigation.gyro.getYaw(), lineElements[3]);
                        robot.navigation.setRobotOrientation(Double.parseDouble(lineElements[3]),
                                robot.navigation.turnMaxSpeed);
                        DbgLog.msg("ftc9773: Reached target orientation");
                    }
                    turnCounter++;
                }
                else{
                    turnCounter = 0;
                    driveSystem.drive((float) speed, (float) direction);
                }
            }
        }
        fileRW.close();
    }

    public void invokeMethod(String methodName, JSONObject actionObj) {
        switch (methodName) {
            case "claimAbeacon": {
                if (robot.beaconClaimObj.beaconColor != BeaconClaim.BeaconColor.NONE) {
                    // Get the range sensor value and pass it on to method
                    double distanceFromWall = robot.navigation.rangeSensor.getDistance(DistanceUnit.CM);
                    while (distanceFromWall >= 255){
                        curOpMode.sleep(20);
                        distanceFromWall = robot.navigation.rangeSensor.getDistance(DistanceUnit.CM);
                    }
                    DbgLog.msg("ftc9773: Distance from wall = %f", distanceFromWall);
                    robot.beaconClaimObj.claimABeacon(distanceFromWall);
                }
                break;
            }
            case "setBeaconStatus": {
                robot.beaconClaimObj.setBeaconStatus();
                break;
            }
            case "verifyBeaconColor": {
                robot.beaconClaimObj.verifyBeaconColor();
                break;
            }
            case "verifyBeaconServo": {
                robot.beaconClaimObj.verifyBeaconServo();
                break;
            }
            case "printNavigationValues" : {
                robot.navigation.printNavigationValues();
                break;
            }
            case "TurnDegrees": {
                DbgLog.msg("ftc9773: currentYaw = %f", robot.navigation.gyro.getYaw());
                double degrees = 0.0;
                double speed = robot.navigation.turnMaxSpeed;
                try {
                    String key = JsonReader.getRealKeyIgnoreCase(actionObj, "degrees");
                    degrees = actionObj.getDouble(key);
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "motorSpeed");
                    speed = actionObj.getDouble(key);
                } catch (JSONException e) {
                    e.printStackTrace();
                }
                //robot.navigation.navxMicro.turnRobot(degrees, speed, navigationChecks);
                DbgLog.msg("ftc9773: currentYaw = %f", robot.navigation.gyro.getYaw());
                break;
            }
            case "TurnUntilWhiteLine": {
                robot.navigation.lf.turnUntilWhiteLine(false);
                break;
            }
            case "DriveToDistance": {
                double distance = 0.0;
                double speed = robot.navigation.straightDrMaxSpeed;
                try {
                    String key = JsonReader.getRealKeyIgnoreCase(actionObj, "inches");
                    distance = actionObj.getDouble(key);
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "motorSpeed");
                    speed = actionObj.getDouble(key);
                } catch (JSONException e) {
                    e.printStackTrace();
                }
                DbgLog.msg("ftc9773: DriveToDistance: inches=%f, motorSpeed=%f", distance, speed);
                robot.driveSystem.driveToDistance((float) speed, distance);
                break;
            }
            case "SetRobotOrientation": {
                double orientation = 0.0;
                double speed = robot.navigation.turnMaxSpeed;
                try {
                    String key = JsonReader.getRealKeyIgnoreCase(actionObj, "degrees");
                    orientation = actionObj.getDouble(key);
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "motorSpeed");
                    speed = actionObj.getDouble(key);
                } catch (JSONException e) {
                    e.printStackTrace();
                }
                DbgLog.msg("ftc9773: SetRobotOrientation: angle=%f, speed=%f", orientation, speed);
                robot.navigation.setRobotOrientation(orientation, speed);
                break;
            }
            case "printMinMaxLightDetected": {
                robot.navigation.lf.printMinMaxLightDetected();
                break;
            }
            case "reverseDriveSystem": {
                driveSystem.reverse();
                break;
            }
            case "Sleep": {
                int milliseconds = 0;
                try {
                    String key = JsonReader.getRealKeyIgnoreCase(actionObj, "milliSeconds");
                    milliseconds = actionObj.getInt(key);
                } catch (JSONException e) {
                    e.printStackTrace();
                }
                curOpMode.sleep(milliseconds);
                break;
            }
            case "DriveUntilAllianceBeacon": {
                double motorSpeed=0.5;
                double maxDistance1=8.0; // in inches
                double maxDistance2=13.0; // in inches
                double degrees=0.0;
                try {
                    String key = JsonReader.getRealKeyIgnoreCase(actionObj, "degrees");
                    degrees = actionObj.getDouble(key);
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "maxDistance1");
                    maxDistance1 = actionObj.getDouble(key);
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "maxDistance2");
                    maxDistance2 = actionObj.getDouble(key);
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "motorSpeed");
                    motorSpeed = actionObj.getDouble(key);
                } catch (JSONException e) {
                    e.printStackTrace();
                }
                DbgLog.msg("ftc9773: Degrees: %f, maxDistance1: %f, maxDistance2: %f, motorSpeed: %f",
                        degrees, maxDistance1, maxDistance2, motorSpeed);
                robot.navigation.driveUntilAllianceBeacon(motorSpeed, degrees,
                        maxDistance1, maxDistance2);
                break;
            }
            case "startPartAcc":
                robot.partAccObj.activateParticleAccelerator();
                break;
            case "stopPartAcc":
                robot.partAccObj.deactivateParticleAccelerator();
                break;
            case "releaseParticles":
                robot.particleObj.releaseParticles();
                break;
            case "keepParticles":
                robot.particleObj.keepParticles();
                break;
            case "shootParticles": {
                int numParticles=1;
                try {
                    String key = JsonReader.getRealKeyIgnoreCase(actionObj, "numberOfParticles");
                    numParticles = actionObj.getInt(key);
                } catch (JSONException e) {
                    e.printStackTrace();
                }
                DbgLog.msg("ftc9773: shootParticles: numberOfParticles to shoot = %d", numParticles);
                if (numParticles > 0) {
                    robot.particleObj.releaseParticles();
                    curOpMode.sleep(500);
                    robot.particleObj.keepParticles();
                    curOpMode.sleep(1200);
                    if (numParticles > 1) {
                        robot.particleObj.releaseParticles();
                        curOpMode.sleep(1200);
                        robot.particleObj.keepParticles();
                    }
                }
                break;
            }
            case "GoStraightToDistance": {
                double inches = 0.0;
                double motorSpeed = 0.0;
                double degrees=0.0;
                try {
                    String key = JsonReader.getRealKeyIgnoreCase(actionObj, "degrees");
                    degrees = actionObj.getDouble(key);
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "inches");
                    inches = actionObj.getDouble(key);
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "motorSpeed");
                    motorSpeed = actionObj.getDouble(key);
                } catch (JSONException e) {
                    e.printStackTrace();
                }
                DbgLog.msg("ftc9773: Degrees: %f, inches: %f, motorSpeed: %f", degrees, inches, motorSpeed);
                robot.navigation.goStraightToDistance(inches, degrees, (float) motorSpeed);
                break;
            }
            case "GoStraightToWhiteLine": {
                double motorSpeed = 0.0;
                double degrees=0.0;
                boolean driveBackwards = false;
                try {
                    String key = JsonReader.getRealKeyIgnoreCase(actionObj, "degrees");
                    degrees = actionObj.getDouble(key);
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "motorSpeed");
                    motorSpeed = actionObj.getDouble(key);
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "driveBackwards");
                    driveBackwards = actionObj.getBoolean(key);
                } catch (JSONException e) {
                    e.printStackTrace();
                }
                robot.navigation.goStraightToWhiteLine(degrees, (float) motorSpeed, driveBackwards);
                break;
            }
            case "GoStriaghtTillGyroIsStable" : {
                double inches = 0.0;
                double motorSpeed = 0.0;
                double degrees=0.0;
                double degreeTolerance=2.0;
                int numUpdatesToSettle = 5;
                try {
                    String key = JsonReader.getRealKeyIgnoreCase(actionObj, "degrees");
                    degrees = actionObj.getDouble(key);
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "degreeTolerance");
                    degreeTolerance = actionObj.getDouble(key);
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "inches");
                    inches = actionObj.getDouble(key);
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "motorSpeed");
                    motorSpeed = actionObj.getDouble(key);
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "numUpdatesToSettle");
                    numUpdatesToSettle = actionObj.getInt(key);
                } catch (JSONException e) {
                    e.printStackTrace();
                }
                DbgLog.msg("ftc9773: Degrees: %f, inches: %f, motorSpeed: %f", degrees, inches, motorSpeed);
                robot.navigation.goStraightTillGyroIsStable(inches, degrees, (float)degreeTolerance,
                        (float) motorSpeed, numUpdatesToSettle);
                break;
            }
            case "navxGoStraightPID": {
                double degrees = 0;
                String termCondition = null;
                double inches = 0.0;
                double speed=0.5;
                boolean driveUntilWhiteLine = false;
                boolean driveBackwards = false;
                try {
                    String key = JsonReader.getRealKeyIgnoreCase(actionObj, "degrees");
                    degrees = actionObj.getDouble(key);
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "endingCondition");
                    termCondition = actionObj.getString(key);
                    if (termCondition.equalsIgnoreCase("driveToDistance")) {
                        key = JsonReader.getRealKeyIgnoreCase(actionObj, "inches");
                        inches = actionObj.getDouble(key);
                    } else if (termCondition.equalsIgnoreCase("driveUntilWhiteLine")) {
                        driveUntilWhiteLine = true;
                    }
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "driveBackwards");
                    driveBackwards = actionObj.getBoolean(key);
                } catch (JSONException e) {
                    e.printStackTrace();
                }
                DbgLog.msg("ftc9773: degrees=%f, inches=%f, driveBackwards=%b", degrees, inches, driveBackwards);
                if (driveUntilWhiteLine) {
                    while ((!robot.navigation.lf.onWhiteLine()) && robot.curOpMode.opModeIsActive()) {
                        robot.navigation.gyro.goStraightPID(driveBackwards, degrees, (float) speed);
                    }
                    driveSystem.stop();
                } else {
                    // drive to distance using navx go straight pid controller
                    DriveSystem.ElapsedEncoderCounts elapsedCounts =
                            driveSystem.getNewElapsedCountsObj();
                    elapsedCounts.reset();
                    while ((elapsedCounts.getDistanceTravelledInInches() < inches) &&
                            robot.curOpMode.opModeIsActive()) {
                        robot.navigation.gyro.goStraightPID(driveBackwards, degrees, (float) speed);
                    }
                    driveSystem.stop();
                }
                break;
            }
            case "shiftRobot": {
                double shiftDistance = 0.0;
                double moveDistance = 0.0;
                double motorSpeed = 1.0;
                double startingYaw = 0.0;
                double endingYaw = 0.0;
                boolean returnToSamePos=false;
                try {
                    String key = JsonReader.getRealKeyIgnoreCase(actionObj, "shiftDistance");
                    shiftDistance = actionObj.getDouble(key);
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "moveDistance");
                    moveDistance = actionObj.getDouble(key);
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "returnToSamePos");
                    returnToSamePos = actionObj.getBoolean(key);
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "motorSpeed");
                    motorSpeed = actionObj.getDouble(key);
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "startingYaw");
                    startingYaw = actionObj.getDouble(key);
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "endingYaw");
                    endingYaw = actionObj.getDouble(key);
                } catch (JSONException e) {
                    e.printStackTrace();
                }
                DbgLog.msg("ftc9773: shiftDistance=%f, moveDistance=%f, motorSpeed=%f, startingYaw=%f, endingYaw=%f, returnToSamePos=%b",
                        shiftDistance, moveDistance, motorSpeed, startingYaw, endingYaw, returnToSamePos);
                robot.navigation.shiftRobot(shiftDistance, moveDistance, motorSpeed, returnToSamePos, startingYaw, endingYaw);
                break;
            }
            case "shiftToWall": {
                double targetDistance = 0.0; // in cm
                double moveDistance = 0.0;
                double motorSpeed = 1.0;
                boolean returnToSamePos=false;
                double distanceFromWall; // in cm
                double distTolerance=2.0; // in cm
                double distanceToShift=0.0;
                try{
                    String key = JsonReader.getRealKeyIgnoreCase(actionObj, "targetDistance");
                    targetDistance = actionObj.getDouble(key);
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "distTolerance");
                    distTolerance = actionObj.getDouble(key);
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "moveDistance");
                    moveDistance = actionObj.getDouble(key);
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "returnToSamePos");
                    returnToSamePos = actionObj.getBoolean(key);
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "motorSpeed");
                    motorSpeed = actionObj.getDouble(key);
                }
                catch (JSONException e){
                    e.printStackTrace();
                }
                distanceFromWall = robot.navigation.rangeSensor.getDistance(DistanceUnit.CM);
                while (distanceFromWall >= 255){
                    curOpMode.sleep(20);
                    distanceFromWall = robot.navigation.rangeSensor.getDistance(DistanceUnit.CM);
                }
                if (Math.abs(targetDistance - distanceFromWall) > distTolerance) {
                    distanceToShift = CM2INCHES * (targetDistance - distanceFromWall);
                    DbgLog.msg("ftc9773: targetDistance=%f cm, moveDistance=%f, distanceFromWall=%f cm, tolerance = %f cm, distanceToShift=%f inches, motorSpeed=%f, returnToSamePos=%b",
                            targetDistance, moveDistance, distanceFromWall, distTolerance,
                            distanceToShift, motorSpeed, returnToSamePos);
                    double startingYaw = (allianceColor.equalsIgnoreCase("red") ? 0.0 : 180);
                    double endingYaw = startingYaw;
                    robot.navigation.shiftRobot(distanceToShift, moveDistance, motorSpeed, returnToSamePos, startingYaw, endingYaw);
                } else {
                    DbgLog.msg("ftc9773: No need to shift; current distance from wall is within the tolerance!");
                    DbgLog.msg("ftc9773: targetDistance=%f cm, moveDistance=%f, distanceFromWall=%f cm, tolerance = %f cm, distanceToShift=%f inches, motorSpeed=%f, returnToSamePos=%b",
                            targetDistance, moveDistance, distanceFromWall, distTolerance,
                            distanceToShift, motorSpeed, returnToSamePos);
                }
                break;
            }
            case "testEncoders":{
                robot.driveSystem.testEncoders();
            }
        }
    }

    public void doActions() throws InterruptedException {
        int len = autoCfg.actions.length();
        JSONObject actionObj;

        String replayFile;
        String methodName;
//        DbgLog.msg("ftc9773: Number of autonomous actions = %d", len);
        for (int i =0; i<len && curOpMode.opModeIsActive(); i++) {
            DbgLog.msg("ftc9773: i=%d", i);
            try {
                actionObj = autoCfg.getAction(i);
                String key = JsonReader.getRealKeyIgnoreCase(actionObj, "type");
                if (actionObj.getString(key).equalsIgnoreCase("Replay")) {
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "value");
                    replayFile = this.replayFilesDir + actionObj.getString(key);
                    DbgLog.msg("ftc9773: Replaying the file %s", replayFile);
                    replayFileAction(replayFile);
                }
                else if (actionObj.getString(key).equalsIgnoreCase("Programmed")) {
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "value");
                    methodName = actionObj.getString(key);
                    DbgLog.msg("ftc9773: Invoking method: %s", methodName);
                    invokeMethod(methodName,actionObj);
                }
            } catch (JSONException e) {
                e.printStackTrace();
            }
        }
    }


}
