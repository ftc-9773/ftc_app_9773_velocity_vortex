package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.attachments.Attachment;
import org.firstinspires.ftc.teamcode.attachments.BeaconClaim;
import org.firstinspires.ftc.teamcode.attachments.CapBallLift;
import org.firstinspires.ftc.teamcode.attachments.Harvester;
import org.firstinspires.ftc.teamcode.drivesys.DriveSystem;
import org.firstinspires.ftc.teamcode.navigation.Navigation;
import org.firstinspires.ftc.teamcode.navigation.NavxMicro;
import org.firstinspires.ftc.teamcode.util.FileRW;
import org.firstinspires.ftc.teamcode.util.JsonReaders.JsonReader;
import org.firstinspires.ftc.teamcode.util.JsonReaders.RobotConfigReader;
import org.json.JSONException;
import org.json.JSONObject;

import java.util.concurrent.TimeUnit;

import static java.lang.Thread.sleep;


public class FTCRobot {
    public LinearOpMode curOpMode;
    public DriveSystem driveSystem=null;
    public Navigation navigation =null;
    public Attachment[] attachmentsArr;
    public AutonomousActions autonomousActions;
    public BeaconClaim beaconClaimObj;
    public CapBallLift capBallLiftObj;
    public Harvester harvesterObj;

    public FTCRobot(LinearOpMode curOpMode, String robotName) {
        this.curOpMode = curOpMode;
        RobotConfigReader robotConfig;
        robotConfig = new RobotConfigReader(JsonReader.baseDir+"robots.json",  robotName);
        String driveSysName = null;

        // Instantiate the Drive System
        try {
            driveSysName = robotConfig.getDriveSysName();
            if (driveSysName != null) {
                DbgLog.msg("driveSysName = %s", driveSysName);
                driveSystem = DriveSystem.createDriveSystem(curOpMode, this, driveSysName);
            }
        }catch (Exception e){
            e.printStackTrace();
        }
        if (driveSystem == null) {
            DbgLog.error("Drivesystem not properly initialized");
            DbgLog.msg("");
        }

        // Create the objects for attachments
        createAttachments(robotConfig.getAttachments());

        // Initialize navigation subsystem
        if (robotConfig.getNavigationOption() != null) {
            navigation = new Navigation(this, curOpMode, robotConfig.getNavigationOption());
        } else{
            navigation = null;
        }
    }

    public void createAttachments (String[] attachments) {
        JsonReader attachmentsReader = new JsonReader(JsonReader.attachments);
        JSONObject rootObj = attachmentsReader.jsonRoot;
        attachmentsArr = new Attachment[attachments.length];
        for (int i=0; i<attachments.length; i++) {
            if (attachments[i].equals("BeaconClaim")) {
                attachmentsArr[i] = new BeaconClaim(this, curOpMode, rootObj);
                beaconClaimObj = (BeaconClaim) attachmentsArr[i];
                DbgLog.msg("beaconClaimObj created");
            }
            else if (attachments[i].equals("CapBallLift")) {
                attachmentsArr[i] = new CapBallLift(this, curOpMode, rootObj);
                capBallLiftObj = (CapBallLift) attachmentsArr[i];
                DbgLog.msg("capBallLiftObj created");
            }
            else if (attachments[i].equals("Harvester")){
                attachmentsArr[i] = new Harvester(this, curOpMode, rootObj);
                harvesterObj = (Harvester) attachmentsArr[i];
                DbgLog.msg("harvesterObj created");
            }
        }
        return;
    }

    public void runTeleOp(String allianceColor) {
        float speed;
        float direction;

        curOpMode.waitForStart();
        while(curOpMode.opModeIsActive()){
            speed = -curOpMode.gamepad1.left_stick_y;
            direction = curOpMode.gamepad1.right_stick_x;

            driveSystem.drive(speed, direction);
            for (int i=0; i<attachmentsArr.length; i++) {
                attachmentsArr[i].getAndApplyDScmd();
            }

            curOpMode.idle();
        }
        return;
    }

    public void runAutonomous(String autonomousOpt, String allianceColor,
                                           long startingDelay, int startingPosition) {
        this.autonomousActions =
                new AutonomousActions(this, curOpMode, autonomousOpt, allianceColor);

        try {
            curOpMode.waitForStart();
            DbgLog.msg("Starting delay = %d seconds", startingDelay);
            if (startingDelay > 0) {
                sleep(startingDelay);
            }
            while (curOpMode.opModeIsActive()) {
                autonomousActions.doActions();
                break;
            }
            driveSystem.stop();
            curOpMode.stop();
        } catch (InterruptedException e) {
            driveSystem.stop();
            curOpMode.stop();
        }
        return;
    }

    public void autonomousRecord(JsonReader opmodeCfg, String allianceColor) throws InterruptedException {
        long clockCycle=5000;
        String recordFilePath, recordFileName=null;
        FileRW fileRW;
        String recordFilesDir=null;
        if (allianceColor.equalsIgnoreCase("red"))
            recordFilesDir = JsonReader.autonomousRedDir;
        else if (allianceColor.equalsIgnoreCase("blue"))
            recordFilesDir = JsonReader.autonomousBlueDir;

        try {
            String key = JsonReader.getRealKeyIgnoreCase(opmodeCfg.jsonRoot, "recordFileName");
            recordFileName = opmodeCfg.jsonRoot.getString(key);
            clockCycle = opmodeCfg.jsonRoot.getLong("clock-cycle");
            clockCycle = clockCycle * 1000000; // convert milli seconds into nano seconds
        }
        catch (JSONException exc) {
            exc.printStackTrace();
        }
        recordFilePath = new String(recordFilesDir + recordFileName);

        DbgLog.msg("clock Cycle = %d nanoseconds", clockCycle);
        DbgLog.msg("record filepath = %s", recordFilePath);

        fileRW = new FileRW(recordFilePath, true);
        // First row is a header row.
        String firstrow = new String("elapsedTimeNanoSec, speed, direction");
        fileRW.fileWrite(firstrow);

        curOpMode.waitForStart();
        long startingTime = System.nanoTime();
        long elapsedTime=0, prev_elapsedTime = 0;
        long sleepTime = 0;
        double spinAngle = 0;
        while (curOpMode.opModeIsActive()) {
            double speed = -curOpMode.gamepad1.left_stick_y * 0.3;
            double direction = curOpMode.gamepad1.right_stick_x * 0.5;
            if(curOpMode.gamepad1.left_bumper){
                spinAngle = navigation.navxMicro.getModifiedYaw();
            }

            elapsedTime = System.nanoTime() - startingTime;
            driveSystem.drive((float) speed, (float) direction);
            if(spinAngle != 0) {
                fileRW.fileWrite(Long.toString(elapsedTime) + "," + Double.toString(speed) + "," +
                        Double.toString(direction) + "," + Double.toString(spinAngle));
                spinAngle = 0;
            }
            else if(spinAngle == 0){
                fileRW.fileWrite(Long.toString(elapsedTime) + "," + Double.toString(speed) + "," +
                        Double.toString(direction));
            }

            DbgLog.msg(String.format("Speed: %f, Direction: %f", speed, direction));

            if(curOpMode.gamepad1.a){
                break;
            }

            sleepTime = clockCycle - (elapsedTime - prev_elapsedTime);
            if (sleepTime > 0) {
                TimeUnit.NANOSECONDS.sleep(sleepTime);
            }
            prev_elapsedTime = elapsedTime;
            // sleep(5);
        }
        DbgLog.msg("Is close executing?");
        fileRW.close();
    }
}
