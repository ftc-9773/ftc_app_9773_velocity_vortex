package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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
//            DbgLog.msg("lineElements length = %d", lineElements.length);
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
                        DbgLog.msg("Yaw: %f, Target yaw = %s", robot.navigation.navxMicro.getModifiedYaw(), lineElements[3]);
                        robot.navigation.setRobotOrientation(Double.parseDouble(lineElements[3]),
                                robot.navigation.turnMaxSpeed);
                        DbgLog.msg("Reached target orientation");
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
                robot.beaconClaimObj.claimABeacon();
                break;
            }
            case "claimAbeaconOld": {
                int beaconId = 1;
                try {
                    String key = JsonReader.getRealKeyIgnoreCase(actionObj, "BeaconId");
                    beaconId = actionObj.getInt(key);
                } catch (JSONException e) {
                    e.printStackTrace();
                }
                robot.beaconClaimObj.claimABeaconOld(beaconId);
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
                DbgLog.msg("currentYaw = %f", robot.navigation.navxMicro.getModifiedYaw());
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
                DbgLog.msg("currentYaw = %f", robot.navigation.navxMicro.getModifiedYaw());
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
                boolean driveBackwards=false;
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
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "driveBackwards");
                    driveBackwards = actionObj.getBoolean(key);
                } catch (JSONException e) {
                    e.printStackTrace();
                }
                DbgLog.msg("Degrees: %f, maxDistance1: %f, maxDistance2: %f, motorSpeed: %f, driveBackwards: %b",
                        degrees, maxDistance1, maxDistance2, motorSpeed, driveBackwards);
                robot.navigation.driveUntilAllianceBeacon(driveBackwards, motorSpeed, degrees,
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
            case "GoStraightToDistance": {
                double inches = 0.0;
                double motorSpeed = 0.0;
                double degrees=0.0;
                boolean driveBackwards = false;
                try {
                    String key = JsonReader.getRealKeyIgnoreCase(actionObj, "degrees");
                    degrees = actionObj.getDouble(key);
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "inches");
                    inches = actionObj.getDouble(key);
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "motorSpeed");
                    motorSpeed = actionObj.getDouble(key);
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "driveBackwards");
                    driveBackwards = actionObj.getBoolean(key);
                } catch (JSONException e) {
                    e.printStackTrace();
                }
                DbgLog.msg("Degrees: %f, inches: %f, motorSpeed: %f, driveBackwards: %b", degrees, inches, motorSpeed, driveBackwards);
                robot.navigation.goStraightToDistance(inches, degrees, (float) motorSpeed, driveBackwards);
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
            case "navxGoStraightPID": {
                double Kp = 0.005;
                double degrees = 0;
                String termCondition = null;
                double inches = 0.0;
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
                Kp = robot.navigation.navxMicro.straightPID_kp;
                DbgLog.msg("degrees=%f, kp=%f, inches=%f, driveBackwards=%b", degrees, Kp, inches, driveBackwards);
                if (driveUntilWhiteLine) {
                    while ((!robot.navigation.lf.onWhiteLine()) && robot.curOpMode.opModeIsActive()) {
                        robot.navigation.navxMicro.navxGoStraightPID(driveBackwards, degrees);
                    }
                    driveSystem.stop();
                } else {
                    // drive to distance using navx go straight pid controller
                    DriveSystem.ElapsedEncoderCounts elapsedCounts =
                            driveSystem.getNewElapsedCountsObj();
                    elapsedCounts.reset();
                    while ((elapsedCounts.getDistanceTravelledInInches() < inches) &&
                            robot.curOpMode.opModeIsActive()) {
                        robot.navigation.navxMicro.navxGoStraightPID(driveBackwards, degrees);
                    }
                    driveSystem.stop();
                }
                break;
            }
            case "shiftRobot": {
                double distance = 0.0;
                double moveDistance = 0.0;
                double motorSpeed = 1.0;
                boolean isForward = false;
                try {
                    String key = JsonReader.getRealKeyIgnoreCase(actionObj, "distance");
                    distance = actionObj.getDouble(key);
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "moveDistance");
                    moveDistance = actionObj.getDouble(key);
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "isForward");
                    isForward = actionObj.getBoolean(key);
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "motorSpeed");
                    motorSpeed = actionObj.getDouble(key);
                } catch (JSONException e) {
                    e.printStackTrace();
                }
                robot.navigation.shiftRobot(distance, moveDistance, isForward, motorSpeed);
                break;
            }
            case "shiftToWall": {
                double targetDistance = 0.0;
                double moveDistance = 0.0;
                double motorSpeed = 1.0;
                boolean isForward = false;
                double distanceFromWall;
                double distanceToShift;
                try{
                    String key = JsonReader.getRealKeyIgnoreCase(actionObj, "targetDistance");
                    targetDistance = actionObj.getDouble(key);
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "moveDistance");
                    moveDistance = actionObj.getDouble(key);
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "isForward");
                    isForward = actionObj.getBoolean(key);
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "motorSpeed");
                    motorSpeed = actionObj.getDouble(key);
                }
                catch (JSONException e){
                    e.printStackTrace();
                }
                distanceFromWall = robot.navigation.rangeSensor.getDistance(DistanceUnit.INCH);
                distanceToShift = distanceFromWall - targetDistance;
                DbgLog.msg("targetDistance=%f, moveDistance=%f, distanceFromWall=%f, distanceToShift=%f, motorSpeed=%f, isForward=%b",
                        targetDistance, moveDistance, distanceFromWall, distanceToShift, motorSpeed, isForward);
                // If we are already close enough to the wall, then do nothing.
                if (distanceToShift > 0) {
                    if (allianceColor.equalsIgnoreCase("red")) {
                        // shift left
                        robot.navigation.shiftRobot(-distanceToShift, moveDistance, isForward, motorSpeed);
                    } else if (allianceColor.equalsIgnoreCase("blue")) {
                        // shift right
                        robot.navigation.shiftRobot(distanceToShift, moveDistance, isForward, motorSpeed);
                    }
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
//        DbgLog.msg("Number of autonomous actions = %d", len);
        for (int i =0; i<len && curOpMode.opModeIsActive(); i++) {
            DbgLog.msg("i=%d", i);
            try {
                actionObj = autoCfg.getAction(i);
                String key = JsonReader.getRealKeyIgnoreCase(actionObj, "type");
                if (actionObj.getString(key).equalsIgnoreCase("Replay")) {
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "value");
                    replayFile = this.replayFilesDir + actionObj.getString(key);
                    DbgLog.msg("Replaying the file %s", replayFile);
                    replayFileAction(replayFile);
                }
                else if (actionObj.getString(key).equalsIgnoreCase("Programmed")) {
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "value");
                    methodName = actionObj.getString(key);
                    DbgLog.msg("Invoking method: %s", methodName);
                    invokeMethod(methodName,actionObj);
                }
            } catch (JSONException e) {
                e.printStackTrace();
            }
        }
    }


}
