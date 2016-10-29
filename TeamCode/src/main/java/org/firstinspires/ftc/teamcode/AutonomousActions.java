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
        fileRW.getNextLine(); // skip the header row
        String line;
        long startingTime = System.nanoTime();
        long elapsedTime = 0;
        long sleepTime = 0;
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
                driveSystem.drive((float) speed, (float) direction);
            }
        }
        fileRW.close();
    }

    public void invokeMethod(String methodName) {
        // ToDo: invoke the findWhiteLine method
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

                stopLineFollow = robot.beaconClaimObj.rangeSensorLessThanMinimum(10) ||
                        robot.navigation.lf.timeoutReached();
            }
            DbgLog.msg("Done with lineFollow");
            robot.driveSystem.stop();
        }
        else if(methodName.equalsIgnoreCase("claimAbeacon")){
            robot.beaconClaimObj.claimABeacon();
        }
        else if(methodName.equalsIgnoreCase("verifyBeaconColor")){
            robot.beaconClaimObj.verifyBeaconColor();
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
                    curOpMode.telemetry.addData("Invoking method: %s", methodName);
                    invokeMethod(methodName);
                }
            } catch (JSONException e) {
                e.printStackTrace();
            }
        }
    }
}
