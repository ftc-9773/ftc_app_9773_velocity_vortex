package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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
        String recordType = fileRW.getNextLine();
        fileRW.getNextLine();
        String line;
        switch (recordType){
            case "fusion":
                long startingTime = System.nanoTime();
                long elapsedTime = 0;
                long sleepTime = 0;
                int turnCounter = 0;
                while ((line = fileRW.getNextLine()) != null) {
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
                                robot.navigation.navxMicro.setRobotOrientation(Double.parseDouble(lineElements[3]));
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
                break;
            case "sensor":
                while ((line = fileRW.getNextLine()) != null){
                    String[] lineElements = line.split(",");

                    switch (lineElements[0]){
                        case "encoder":
                            int[] targetPosition = new int[lineElements.length - 1];
                            for (int i=0;i<targetPosition.length;i++){
                                targetPosition[i] = Integer.parseInt(lineElements[i+1]);
                            }

                            driveSystem.driveMotorsToDistance(targetPosition);
                            break;
                        case "yaw":
                            robot.navigation.navxMicro.setRobotOrientation(Double.parseDouble(lineElements[1]));
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
        else if (methodName.equalsIgnoreCase("lineFollow")) {
            boolean stopLineFollow = false;
            long elapsedTime = 0;
            robot.navigation.lf.setStartTimeStamp();
            // ToDo: while the touch sensor is not pressed, keep invoking the line follow
            while (!stopLineFollow) {
                robot.navigation.lf.followLine();

                stopLineFollow = robot.beaconClaimObj.touchSensorPressed() ||
                        robot.navigation.lf.timeoutReached();
            }
            DbgLog.msg("Done with lineFollow");
            robot.driveSystem.stop();
        }
        else if (methodName.equalsIgnoreCase("lineFollowProportional")) {
            boolean stopLineFollow = false;
            DbgLog.msg("minDistance=%f", robot.navigation.minDistance);

            robot.beaconClaimObj.activateButtonServo(); // extend the arm for sensing the color
            while (!stopLineFollow) {
                robot.navigation.lf.followLineProportional();
                stopLineFollow = (robot.navigation.rangeSensor.cmUltrasonic() <=
                        robot.navigation.minDistance) ||
                        (robot.beaconClaimObj.isBeaconRed() || robot.beaconClaimObj.isBeaconBlue());
                DbgLog.msg("Range sensor value = %f", robot.navigation.rangeSensor.cmUltrasonic());
            }
            DbgLog.msg("Done with lineFollowProportional");
            robot.driveSystem.stop();
        }
        else if(methodName.equalsIgnoreCase("claimAbeacon")){
            robot.beaconClaimObj.claimABeacon();
        }
        else if(methodName.equalsIgnoreCase("verifyBeaconColor")){
            robot.beaconClaimObj.verifyBeaconColor();
        }
        else if (methodName.equalsIgnoreCase("checkBeaconColor")) {
            robot.beaconClaimObj.checkBeaconColor();
        }
        else if (methodName.equalsIgnoreCase("testSetRobotOrientation")){
            robot.navigation.navxMicro.setRobotOrientation(270);
        }
        else if(methodName.equalsIgnoreCase("moveBackFor1s")){
            while (robot.navigation.rangeSensor.cmUltrasonic() < 25){
                robot.driveSystem.drive((float) -0.3,0);
            }
            driveSystem.stop();
        }
        else if (methodName.equalsIgnoreCase("TurnDegrees")){
            DbgLog.msg("currentYaw = %f", robot.navigation.navxMicro.getModifiedYaw());
            double methodParam = 0.0;
            try {
                String key = JsonReader.getRealKeyIgnoreCase(actionObj, "param");
                methodParam = actionObj.getDouble(key);
            } catch (JSONException e) {
                e.printStackTrace();
            }
            robot.navigation.navxMicro.turnRobot(methodParam);
            DbgLog.msg("currentYaw = %f", robot.navigation.navxMicro.getModifiedYaw());
        }
        else if(methodName.equalsIgnoreCase("TurnUntilWhiteLine")){
            robot.navigation.lf.turnUntilWhiteLine(false);
        }
        else if(methodName.equalsIgnoreCase("DriveToDistance")){
            double distance = 0.0;
            try{
                String key = JsonReader.getRealKeyIgnoreCase(actionObj, "param");
                distance = actionObj.getDouble(key);
            }catch (JSONException e) {
                e.printStackTrace();
            }
            robot.driveSystem.driveToDistance((float) robot.navigation.straightDrMaxSpeed, distance);
        }
        else if(methodName.equalsIgnoreCase("DriveUntilWhiteLine")){
            robot.navigation.lf.driveUntilWhiteLine();
        }
        else if(methodName.equalsIgnoreCase("SetRobotOrientation")){
            double orientation = 0.0;
            try{
                String key = JsonReader.getRealKeyIgnoreCase(actionObj, "param");
                orientation = actionObj.getDouble(key);
            }catch (JSONException e) {
                e.printStackTrace();
            }
            robot.navigation.navxMicro.setRobotOrientation(orientation);
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
                String key = JsonReader.getRealKeyIgnoreCase(actionObj, "param");
                milliseconds = actionObj.getInt(key);
            }catch (JSONException e) {
                e.printStackTrace();
            }
            curOpMode.sleep(milliseconds);
        }
    }

    public void doActions() throws InterruptedException {
        int len = autoCfg.actions.length();
        JSONObject actionObj;

        String replayFile;
        String methodName;
//        DbgLog.msg("Number of autonomous actions = %d", len);
        for (int i =0; i<len; i++) {
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
