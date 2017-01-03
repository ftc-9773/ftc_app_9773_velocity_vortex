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
                        DbgLog.msg("ftc9773: Yaw: %f, Target yaw = %s", robot.navigation.navxMicro.getModifiedYaw(), lineElements[3]);
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
                DbgLog.msg("ftc9773: currentYaw = %f", robot.navigation.navxMicro.getModifiedYaw());
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
                DbgLog.msg("ftc9773: currentYaw = %f", robot.navigation.navxMicro.getModifiedYaw());
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
            case "navxGoStraightPID": {
                double Kp = 0.005;
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
                Kp = robot.navigation.navxMicro.straightPID_kp;
                DbgLog.msg("ftc9773: degrees=%f, kp=%f, inches=%f, driveBackwards=%b", degrees, Kp, inches, driveBackwards);
                if (driveUntilWhiteLine) {
                    while ((!robot.navigation.lf.onWhiteLine()) && robot.curOpMode.opModeIsActive()) {
                        robot.navigation.navxMicro.navxGoStraightPID(driveBackwards, degrees, (float) speed);
                    }
                    driveSystem.stop();
                } else {
                    // drive to distance using navx go straight pid controller
                    DriveSystem.ElapsedEncoderCounts elapsedCounts =
                            driveSystem.getNewElapsedCountsObj();
                    elapsedCounts.reset();
                    while ((elapsedCounts.getDistanceTravelledInInches() < inches) &&
                            robot.curOpMode.opModeIsActive()) {
                        robot.navigation.navxMicro.navxGoStraightPID(driveBackwards, degrees, (float) speed);
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
                boolean returnToSamePos=false;
                try {
                    String key = JsonReader.getRealKeyIgnoreCase(actionObj, "distance");
                    distance = actionObj.getDouble(key);
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "moveDistance");
                    moveDistance = actionObj.getDouble(key);
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "isForward");
                    isForward = actionObj.getBoolean(key);
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "returnToSamePos");
                    returnToSamePos = actionObj.getBoolean(key);
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "motorSpeed");
                    motorSpeed = actionObj.getDouble(key);
                } catch (JSONException e) {
                    e.printStackTrace();
                }
                robot.navigation.shiftRobot(distance, moveDistance, isForward, motorSpeed, returnToSamePos);
                break;
            }
            case "shiftToWall": {
                double targetDistance = 0.0;
                double moveDistance = 0.0;
                double motorSpeed = 1.0;
                boolean isForward = false;
                boolean returnToSamePos=false;
                double distanceFromWall;
                double distanceToShift;
                try{
                    String key = JsonReader.getRealKeyIgnoreCase(actionObj, "targetDistance");
                    targetDistance = actionObj.getDouble(key);
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "moveDistance");
                    moveDistance = actionObj.getDouble(key);
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "isForward");
                    isForward = actionObj.getBoolean(key);
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "returnToSamePos");
                    returnToSamePos = actionObj.getBoolean(key);
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "motorSpeed");
                    motorSpeed = actionObj.getDouble(key);
                }
                catch (JSONException e){
                    e.printStackTrace();
                }
                distanceFromWall = robot.navigation.rangeSensor.getDistance(DistanceUnit.INCH);
                distanceToShift = distanceFromWall - targetDistance;
                DbgLog.msg("ftc9773: targetDistance=%f, moveDistance=%f, distanceFromWall=%f, distanceToShift=%f, motorSpeed=%f, isForward=%b, returnToSamePos=%b",
                        targetDistance, moveDistance, distanceFromWall, distanceToShift, motorSpeed, isForward, returnToSamePos);
                robot.navigation.shiftRobot(-distanceToShift, moveDistance, isForward, motorSpeed, returnToSamePos);
                // If we are already close enough to the wall, then do nothing.
//                if (distanceToShift > 0) {
//                    if (allianceColor.equalsIgnoreCase("red")) {
                        // shift left
//                        robot.navigation.shiftRobot(-distanceToShift, moveDistance, isForward, motorSpeed, returnToSamePos);
//                    } else if (allianceColor.equalsIgnoreCase("blue")) {
//                         shift right
//                        robot.navigation.shiftRobot(distanceToShift, moveDistance, isForward, motorSpeed, returnToSamePos);
//                    }
//                }
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
