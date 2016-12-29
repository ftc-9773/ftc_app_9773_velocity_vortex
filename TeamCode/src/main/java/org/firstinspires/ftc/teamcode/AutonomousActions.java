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
            case "searchForWhiteLine": {
                try {
                    robot.navigation.lf.searchForWhiteLine();
                } catch (NullPointerException exc) {
                    exc.printStackTrace();
                    DbgLog.error("Navigation or Line follow object is null");
                }
                break;
            }
            case "lineFollowProportional": {
                boolean stopLineFollow = false;
                DbgLog.msg("minDistance=%f", robot.navigation.minDistance);

//            robot.beaconClaimObj.activateButtonServo(); // extend the arm for sensing the color
                driveSystem.setMaxSpeed((float) robot.navigation.lfMaxSpeed);
                while (!stopLineFollow && curOpMode.opModeIsActive()) {
                    robot.navigation.lf.followLineProportional();
                    stopLineFollow = robot.navigation.rangeSensor.getDistance(DistanceUnit.CM) <=
                            robot.navigation.minDistance;
                    DbgLog.msg("Range sensor value = %f", robot.navigation.rangeSensor.getDistance(DistanceUnit.CM));
                }
                DbgLog.msg("Done with lineFollowProportional");
                if (robot.beaconClaimObj.isBeaconBlue()) {
                    DbgLog.msg("Blue detected");
                } else if (robot.beaconClaimObj.isBeaconRed()) {
                    DbgLog.msg("Red detected");
                }
                driveSystem.stop();
                driveSystem.resumeMaxSpeed();
                break;
            }
            case "claimAbeacon": {
                int beaconId = 1;
                try {
                    String key = JsonReader.getRealKeyIgnoreCase(actionObj, "BeaconId");
                    beaconId = actionObj.getInt(key);
                } catch (JSONException e) {
                    e.printStackTrace();
                }
                robot.beaconClaimObj.claimABeacon(beaconId);
                break;
            }
            case "claimAbeaconV2": {
                robot.beaconClaimObj.claimABeaconV2();
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
            case"checkBeaconColor": {
                robot.beaconClaimObj.checkBeaconColor();
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
            case "DriveUntilWhiteLine": {
                double speed = robot.navigation.straightDrMaxSpeed;
                long timeoutMillis = 5000;
                try {
                    String key = JsonReader.getRealKeyIgnoreCase(actionObj, "motorSpeed");
                    speed = actionObj.getDouble(key);
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "timeoutMillis");
                    timeoutMillis = actionObj.getLong(key);
                } catch (JSONException e) {
                    e.printStackTrace();
                }
                DbgLog.msg("motorSpeed=%f, timeoutMillis=%d", speed, timeoutMillis);
                robot.navigation.lf.driveUntilWhiteLine(speed, timeoutMillis);
                break;
            }
            case "DrivePastWhiteLine": {
                double speed = robot.navigation.straightDrMaxSpeed;
                try {
                    String key = JsonReader.getRealKeyIgnoreCase(actionObj, "motorSpeed");
                    speed = actionObj.getDouble(key);
                } catch (JSONException e) {
                    e.printStackTrace();
                }
                robot.navigation.lf.drivePastWhiteLine(speed);
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
            case "NavxSetRobotOrientation": {
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
            case "SetRobotOrientationForBeacon": {
                double orientation = 0.0;
                double speed = robot.navigation.turnMaxSpeed;
                int beaconId = 0;
                try {
                    String key = JsonReader.getRealKeyIgnoreCase(actionObj, "degrees");
                    orientation = actionObj.getDouble(key);
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "beaconId");
                    beaconId = actionObj.getInt(key);
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "motorSpeed");
                    speed = actionObj.getDouble(key);
                } catch (JSONException e) {
                    e.printStackTrace();
                }
                if (robot.beaconClaimObj.numPressesNeeded[beaconId - 1] != 0) {
                    robot.navigation.setRobotOrientation(orientation, speed);
                }
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
            case "DriveUntilBeacon": {
                double distFromWall = 0.0;
                double speed = 0.0;
                int beaconId = 1;
                int numBlueDetected = 0, numRedDetected = 0;
                try {
                    String key = JsonReader.getRealKeyIgnoreCase(actionObj, "distanceFromWall");
                    distFromWall = actionObj.getInt(key);
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "motorSpeed");
                    speed = actionObj.getDouble(key);
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "BeaconId");
                    beaconId = actionObj.getInt(key);
                } catch (JSONException e) {
                    e.printStackTrace();
                }
                DbgLog.msg("DistanceFromWall = %f, speed = %f", distFromWall, speed);

                driveSystem.setMaxSpeed((float) speed);
                while ((robot.navigation.rangeSensor.getDistance(DistanceUnit.CM) > distFromWall) && curOpMode.opModeIsActive()) {
                    driveSystem.drive(1.0f, 0);
                    if (robot.beaconClaimObj.isBeaconBlue()) {
                        numBlueDetected++;
                    } else if (robot.beaconClaimObj.isBeaconRed()) {
                        numRedDetected++;
                    }
                }
                driveSystem.stop();
                driveSystem.resumeMaxSpeed();
                DbgLog.msg("numBlueDetected = %d, numRedDetected = %d", numBlueDetected, numRedDetected);
                robot.beaconClaimObj.verifyBeaconColor();
                robot.beaconClaimObj.setBeaconStatus(beaconId, robot.autonomousActions.allianceColor,
                        numBlueDetected, numRedDetected);
                DbgLog.msg("rangeSensor value = %f", robot.navigation.rangeSensor.getDistance(DistanceUnit.CM));
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
                        robot.navigation.navxMicro.MygoStraightPID(driveBackwards, degrees);
                    }
                    driveSystem.stop();
                } else {
                    // drive to distance using navx go straight pid controller
                    DriveSystem.ElapsedEncoderCounts elapsedCounts =
                            driveSystem.getNewElapsedCountsObj();
                    elapsedCounts.reset();
                    while ((elapsedCounts.getDistanceTravelledInInches() < inches) &&
                            robot.curOpMode.opModeIsActive()) {
                        robot.navigation.navxMicro.MygoStraightPID(driveBackwards, degrees);
                    }
                    driveSystem.stop();
                }
                break;
            }
            case "navxGoStraightPIDForBeacon": {
                double Kp = 0.005;
                double degrees = 0;
                String termCondition = null;
                double inches = 0.0;
                boolean driveUntilWhiteLine = false;
                boolean driveBackwards = false;
                int beaconId = 0;
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
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "beaconId");
                    beaconId = actionObj.getInt(key);
                } catch (JSONException e) {
                    e.printStackTrace();
                }
                if (robot.beaconClaimObj.numPressesNeeded[beaconId - 1] != 0) {
                    Kp = robot.navigation.navxMicro.straightPID_kp;
                    DbgLog.msg("degrees=%f, kp=%f, inches=%f, driveBackwards=%b", degrees, Kp, inches, driveBackwards);
                    if (driveUntilWhiteLine) {
                        while ((!robot.navigation.lf.onWhiteLine()) && robot.curOpMode.opModeIsActive()) {
                            robot.navigation.navxMicro.MygoStraightPID(driveBackwards, degrees);
                        }
                        driveSystem.stop();
                    } else {
                        // drive to distance using navx go straight pid controller
                        DriveSystem.ElapsedEncoderCounts elapsedCounts =
                                driveSystem.getNewElapsedCountsObj();
                        elapsedCounts.reset();
                        while ((elapsedCounts.getDistanceTravelledInInches() < inches) &&
                                robot.curOpMode.opModeIsActive()) {
                            robot.navigation.navxMicro.MygoStraightPID(driveBackwards, degrees);
                        }
                        driveSystem.stop();
                    }
                }
                break;
            }
            case "shiftRobot": {
                double distance = 0.0;
                boolean isForward = false;
                try {
                    String key = JsonReader.getRealKeyIgnoreCase(actionObj, "distance");
                    distance = actionObj.getDouble(key);
                    key = JsonReader.getRealKeyIgnoreCase(actionObj, "isForward");
                    isForward = actionObj.getBoolean(key);
                } catch (JSONException e) {
                    e.printStackTrace();
                }
                robot.navigation.shiftRobot(distance, isForward);
                break;
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
