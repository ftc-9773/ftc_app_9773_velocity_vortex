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
                        robot.navigation.navxMicro.setRobotOrientation(Double.parseDouble(lineElements[3]),
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
        // ToDo: Change this into a switch statement
        if (methodName.equals("searchForWhiteLine")) {
            try {
                robot.navigation.lf.searchForWhiteLine();
            } catch (NullPointerException exc) {
                exc.printStackTrace();
                DbgLog.error("Navigation or Line follow object is null");
            }
        }
        else if (methodName.equalsIgnoreCase("lineFollowProportional")) {
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
            if (robot.beaconClaimObj.isBeaconBlue()){
                DbgLog.msg("Blue detected");
            }
            else if(robot.beaconClaimObj.isBeaconRed()){
                DbgLog.msg("Red detected");
            }
            driveSystem.stop();
            driveSystem.resumeMaxSpeed();
        }
        else if(methodName.equalsIgnoreCase("claimAbeacon")){
            int beaconId=1;
            try{
                String key = JsonReader.getRealKeyIgnoreCase(actionObj, "BeaconId");
                beaconId = actionObj.getInt(key);
            }catch (JSONException e) {
                e.printStackTrace();
            }
            robot.beaconClaimObj.claimABeacon(beaconId);
        }
        else if(methodName.equalsIgnoreCase("verifyBeaconColor")){
            robot.beaconClaimObj.verifyBeaconColor();
        }
        else if (methodName.equalsIgnoreCase("verifyBeaconServo")) {
            robot.beaconClaimObj.verifyBeaconServo();
        }
        else if (methodName.equalsIgnoreCase("checkBeaconColor")) {
            robot.beaconClaimObj.checkBeaconColor();
        }
        else if (methodName.equalsIgnoreCase("TurnDegrees")){
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
            robot.navigation.navxMicro.turnRobot(degrees, speed);
            DbgLog.msg("currentYaw = %f", robot.navigation.navxMicro.getModifiedYaw());
        }
        else if(methodName.equalsIgnoreCase("TurnUntilWhiteLine")){
            robot.navigation.lf.turnUntilWhiteLine(false);
        }
        else if(methodName.equalsIgnoreCase("DriveToDistance")){
            double distance = 0.0;
            double speed = robot.navigation.straightDrMaxSpeed;
            try{
                String key = JsonReader.getRealKeyIgnoreCase(actionObj, "inches");
                distance = actionObj.getDouble(key);
                key = JsonReader.getRealKeyIgnoreCase(actionObj, "motorSpeed");
                speed = actionObj.getDouble(key);
            }catch (JSONException e) {
                e.printStackTrace();
            }
            robot.driveSystem.driveToDistance((float) speed, distance);
        }
        else if(methodName.equalsIgnoreCase("DriveUntilWhiteLine")){
            double speed = robot.navigation.straightDrMaxSpeed;
            long timeoutMillis = 5000;
            try{
                String key = JsonReader.getRealKeyIgnoreCase(actionObj, "motorSpeed");
                speed = actionObj.getDouble(key);
                key = JsonReader.getRealKeyIgnoreCase(actionObj, "timeoutMillis");
                timeoutMillis = actionObj.getLong(key);
            }catch (JSONException e) {
                e.printStackTrace();
            }
            DbgLog.msg("motorSpeed=%f, timeoutMillis=%d", speed, timeoutMillis);
            robot.navigation.lf.driveUntilWhiteLine(speed, timeoutMillis);
        }
        else if(methodName.equalsIgnoreCase("DrivePastWhiteLine")){
            double speed = robot.navigation.straightDrMaxSpeed;
            try{
                String key = JsonReader.getRealKeyIgnoreCase(actionObj, "motorSpeed");
                speed = actionObj.getDouble(key);
            }catch (JSONException e) {
                e.printStackTrace();
            }
            robot.navigation.lf.drivePastWhiteLine(speed);
        } else if(methodName.equalsIgnoreCase("SetRobotOrientation")){
            double orientation = 0.0;
            double speed = robot.navigation.turnMaxSpeed;
            try{
                String key = JsonReader.getRealKeyIgnoreCase(actionObj, "degrees");
                orientation = actionObj.getDouble(key);
                key = JsonReader.getRealKeyIgnoreCase(actionObj, "motorSpeed");
                speed = actionObj.getDouble(key);
            }catch (JSONException e) {
                e.printStackTrace();
            }
            robot.navigation.navxMicro.setRobotOrientation(orientation, speed);
        }
        else if(methodName.equalsIgnoreCase("printMinMaxLightDetected")) {
            robot.navigation.lf.printMinMaxLightDetected();
        }
        else if(methodName.equalsIgnoreCase("reverseDriveSystem")) {
            driveSystem.reverse();
        }
        else if(methodName.equalsIgnoreCase("Sleep")){
            int milliseconds = 0;
            try{
                String key = JsonReader.getRealKeyIgnoreCase(actionObj, "milliSeconds");
                milliseconds = actionObj.getInt(key);
            }catch (JSONException e) {
                e.printStackTrace();
            }
            curOpMode.sleep(milliseconds);
        }
        else if(methodName.equalsIgnoreCase("DriveUntilBeacon")){
            double distFromWall = 0.0;
            double speed = 0.0;
            int beaconId=1;
            int numBlueDetected=0, numRedDetected=0;
            try{
                String key = JsonReader.getRealKeyIgnoreCase(actionObj, "distanceFromWall");
                distFromWall = actionObj.getInt(key);
                key = JsonReader.getRealKeyIgnoreCase(actionObj, "motorSpeed");
                speed = actionObj.getDouble(key);
                key = JsonReader.getRealKeyIgnoreCase(actionObj, "BeaconId");
                beaconId = actionObj.getInt(key);
            }catch (JSONException e) {
                e.printStackTrace();
            }
            DbgLog.msg("DistanceFromWall = %f, speed = %f", distFromWall, speed);

            driveSystem.setMaxSpeed((float)speed);
            while ((robot.navigation.rangeSensor.getDistance(DistanceUnit.CM) > distFromWall) && curOpMode.opModeIsActive()){
                driveSystem.drive(1.0f, 0);
                if (robot.beaconClaimObj.isBeaconBlue()){
                    numBlueDetected++;
                }
                else if(robot.beaconClaimObj.isBeaconRed()){
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
        }
        else if (methodName.equalsIgnoreCase("startPartAcc")) {
            robot.partAccObj.activateParticleAccelerator();
        }
        else if (methodName.equalsIgnoreCase("stopPartAcc")) {
            robot.partAccObj.deactivateParticleAccelerator();
        }
        else if (methodName.equalsIgnoreCase("releaseParticles")) {
            robot.particleObj.releaseParticles();
        }
        else if (methodName.equalsIgnoreCase("keepParticles")){
            robot.particleObj.keepParticles();
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
