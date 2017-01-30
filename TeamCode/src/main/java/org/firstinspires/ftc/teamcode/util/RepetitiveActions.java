package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.FTCRobot;
import org.firstinspires.ftc.teamcode.navigation.NavxMicro;

import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;

/**
 * Created by ftcrobocracy on 1/22/17.
 */

public class RepetitiveActions {
    LinearOpMode curOpMode;
    FTCRobot robot;
    public enum RepActionID {LOOP_RUNTIME, RANGESENSOR_CM, NAVX_DEGREES}
    public enum LoopType {DRIVE_TO_DISTANCE, DRIVE_UNTIL_WHITELINE, DRIVE_TILL_BEACON, TURN_ROBOT}
    private List<repActionBaseClass> repActions = new ArrayList<repActionBaseClass>();
    public String loopRuntimeLog, rangeSensorLog, navxLog;

    public class repActionBaseClass {
        public RepActionID actionID;
        public int iterationCount;
        public void startAction() {return;}
        public void repeatAction() {return;}
        public void writeToFile() {return;}
        public void printToConsole() {return;}
        public void closeLog() {return;}
    }

    public class LoopRuntime extends repActionBaseClass {
        ElapsedTime timer;
        double minTime, maxTime, avgTime, totalTime;
        LoopType loopType;
        String logFile;
        FileRW fileObj;

        public LoopRuntime(LoopType loopType) {
            actionID = RepActionID.LOOP_RUNTIME;
            this.loopType = loopType;
            timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
            timer.reset();
            minTime = Integer.MAX_VALUE;
            maxTime = avgTime = totalTime = 0.0;
            iterationCount = 0;
            if (loopRuntimeLog != null) {
                // Create a FileRW object
                logFile = FileRW.getTimeStampedFileName(loopRuntimeLog);
                this.fileObj = new FileRW(logFile, true);
                if (this.fileObj == null) {
                    DbgLog.error("Error! Could not create the file %s", logFile);
                }
            }
        }

        @Override
        public void startAction() {
            timer.reset();
            minTime = Integer.MAX_VALUE;
            maxTime = avgTime = totalTime = 0.0;
            iterationCount = 0;
        }

        @Override
        public void repeatAction() {
            double millis = timer.milliseconds();
            timer.reset();
            if (minTime > millis) {
                minTime = millis;
            }
            if (maxTime < millis) {
                maxTime = millis;
            }
            totalTime += millis;
            iterationCount++;
        }

        @Override
        public void writeToFile() {
            avgTime = totalTime / iterationCount;
            String strToWrite = String.format("ftc9773: Timestamp:%s, totalTime=%f, minTime=%f, " +
                    "avgTime=%f, maxTime=%f, count=%d",
                    new SimpleDateFormat("yyyy.MM.dd.HH.mm.ss").format(new Date()),
                    totalTime, minTime, avgTime, maxTime, iterationCount);
            fileObj.fileWrite(strToWrite);
        }

        @Override
        public void printToConsole() {
            avgTime = totalTime / iterationCount;
            DbgLog.msg("ftc9773: totalTime=%f, minTime=%f, avgTime=%f, maxTime=%f, count=%d",
                    totalTime, minTime, avgTime, maxTime, iterationCount);
        }

        @Override
        public void closeLog() {
            if (this.fileObj != null) {
                DbgLog.msg("ftc9773: RepetitiveActions: Closing loopRuntime fileobj");
                this.fileObj.close();
            }
        }
    }

    public class RangeSensorDistance extends repActionBaseClass {
        public ModernRoboticsI2cRangeSensor rangeSensor;
        ElapsedTime timer;
        private double minDistance, maxDistance, totalDistance, avgDistance;
        public double runningAvg;
        public double runningAvgWeight;
        private double prevDistance;
        boolean printEveryUpdate=true;
        String logFile;
        FileRW fileObj;

        public RangeSensorDistance(ModernRoboticsI2cRangeSensor rangeSensor, double runningAvgWeight, boolean printEveryUpdate) {
            actionID = RepActionID.RANGESENSOR_CM;
            iterationCount = 0;
            this.rangeSensor = rangeSensor;
            this.printEveryUpdate = printEveryUpdate;
            minDistance = Double.MAX_VALUE;
            maxDistance = totalDistance = avgDistance = 0.0;
            prevDistance = 0.0;
            runningAvg = 0.0;
            this.runningAvgWeight = runningAvgWeight;
            timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
            timer.reset();
            if (rangeSensorLog != null) {
                // Create a FileRW object
                logFile = FileRW.getTimeStampedFileName(rangeSensorLog);
                this.fileObj = new FileRW(logFile, true);
                if (this.fileObj == null) {
                    DbgLog.error("Error! Could not create the file %s", logFile);
                }
            }
        }

        @Override
        public void startAction() {
            minDistance = Double.MAX_VALUE;
            maxDistance = totalDistance = avgDistance = 0.0;
            prevDistance = 0.0;
            iterationCount = 0;
            timer.reset();
            if (printEveryUpdate) {
                String strToWrite = String.format("voltage, millis, iteration, cm, runningAvg");
                fileObj.fileWrite(strToWrite);
            }
        }

        @Override
        public void repeatAction() {
            iterationCount++;
            double curDistance = rangeSensor.getDistance(DistanceUnit.CM);
            if (curDistance >= 100) { return; }
            if (runningAvg <= 0.0) { runningAvg = curDistance; }
            if (curDistance < minDistance) { minDistance = curDistance; }
            if (curDistance > maxDistance) { maxDistance = curDistance; }
            totalDistance += curDistance;
            runningAvg = curDistance * runningAvgWeight + (1 - runningAvgWeight) * runningAvg;
            if (printEveryUpdate && (curDistance != prevDistance)) {
                String strToWrite = String.format("%f, %f, %d, %f, %f", robot.getVoltage(),
                        timer.milliseconds(), iterationCount, curDistance, runningAvg);
                fileObj.fileWrite(strToWrite);
                prevDistance = curDistance;
            }
        }

        @Override
        public void printToConsole() {
            avgDistance = totalDistance / iterationCount;
            DbgLog.msg("ftc9773: Starting time=%f, minDistance=%f, maxDistance=%f, avgDistance=%f, count=%d, runningAvg=%f",
                    timer.startTime(), minDistance, maxDistance, avgDistance, iterationCount, runningAvg);
        }

        @Override
        public void writeToFile() {
            avgDistance = totalDistance / iterationCount;
            fileObj.fileWrite(String.format("ftc9773: Starting time=%f, minDistance=%f, maxDistance=%f, avgDistance=%f, count=%d, runningAvg=%f",
                    timer.startTime(), minDistance, maxDistance, avgDistance, iterationCount, runningAvg));
        }

        @Override
        public void closeLog() {
            if (this.fileObj != null) {
                DbgLog.msg("ftc9773: RepetitiveActions: Closing rangeSensor fileobj");
                this.fileObj.close();
            }
        }
    }

    public class NavxDegrees extends repActionBaseClass {
        NavxMicro navxMicro;
        double updateCount, prevUpdateCount;
        ElapsedTime timer;
        boolean printEveryUpdate=true;
        double minDegrees, maxDegrees, totalDegrees, avgDegrees;
        int numUpdates;
        String logFile;
        FileRW fileObj;

        public NavxDegrees(NavxMicro navxMicro, boolean printEveryUpdate) {
            actionID = RepActionID.NAVX_DEGREES;
            iterationCount = 0;
            this.navxMicro = navxMicro;
            timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
            timer.reset();
            updateCount = prevUpdateCount =0;
            this.printEveryUpdate = printEveryUpdate;
            minDegrees = Double.MAX_VALUE;
            maxDegrees = totalDegrees = avgDegrees = 0.0;
            numUpdates = 0;
            if (navxLog != null) {
                // Create a FileRW object
                logFile = FileRW.getTimeStampedFileName(navxLog);
                this.fileObj = new FileRW(logFile, true);
                if (this.fileObj == null) {
                    DbgLog.error("ftc9773: Error! Could not create the file %s", logFile);
                }
            }
        }

        @Override
        public void startAction() {
            timer.reset();
            updateCount = prevUpdateCount = 0;
            iterationCount = 0;
            numUpdates = 0;
            minDegrees = Double.MAX_VALUE;
            maxDegrees = totalDegrees = avgDegrees = 0.0;
            if (printEveryUpdate) {
                String strToWrite = String.format("voltage, millis, iteration, yaw degrees, pitch, updateCount");
                fileObj.fileWrite(strToWrite);
            }
        }

        @Override
        public void repeatAction() {
            double curDegrees = navxMicro.getModifiedYaw();
            iterationCount++;
            if (curDegrees < minDegrees) {
                minDegrees = curDegrees;
            }
            if (curDegrees > maxDegrees) {
                maxDegrees = curDegrees;
            }
            totalDegrees += curDegrees;
            prevUpdateCount = updateCount;
            updateCount = navxMicro.getUpdateCount();
            if (printEveryUpdate && (updateCount != prevUpdateCount)) {
                numUpdates++;
                String strToWrite = String.format("%f, %f, %d, %f, %f, %f", robot.getVoltage(),
                        timer.milliseconds(), iterationCount, curDegrees, navxMicro.getPitch(),
                        updateCount);
                fileObj.fileWrite(strToWrite);
            }
        }

        @Override
        public void printToConsole() {
            avgDegrees = totalDegrees / iterationCount;
            DbgLog.msg("ftc9773: Starting time=%f, minDegrees=%f, maxDegrees=%f, avgDegreese=%f, " +
                    "count=%d, updateCount=%f",
                    timer.startTime(), minDegrees, maxDegrees, avgDegrees, iterationCount, updateCount);
            DbgLog.msg("2 Sensor Rate (Hz)...", Byte.toString(navxMicro.navx_device.getActualUpdateRate()));
            DbgLog.msg("3 Transfer Rate (Hz).", Integer.toString(navxMicro.navx_device.getCurrentTransferRate()));
            DbgLog.msg("4 Delivvered Rate (Hz)", Integer.toString(navxMicro.navx_perfmon.getDeliveredRateHz()));
            DbgLog.msg("5 Missed Samples.....", Integer.toString(navxMicro.navx_perfmon.getNumMissedSensorTimestampedSamples()));
            DbgLog.msg("6 Duplicate Samples..", Integer.toString(navxMicro.navx_device.getDuplicateDataCount()));
            DbgLog.msg("7 Sensor deltaT (ms).", Long.toString(navxMicro.navx_perfmon.getLastSensorTimestampDeltaMS()));
            DbgLog.msg("8 System deltaT (ms).", Long.toString(navxMicro.navx_perfmon.getLastSystemTimestampDeltaMS()));

        }

        @Override
        public void writeToFile() {
            avgDegrees = totalDegrees / iterationCount;
            fileObj.fileWrite(String.format("ftc9773: Starting time=%f, minDegrees=%f, " +
                    "maxDegrees=%f, avgDegreese=%f, count=%d, updateCount=%f",
                    timer.startTime(), minDegrees, maxDegrees, avgDegrees, iterationCount, updateCount));
        }

        @Override
        public void closeLog() {
            if (this.fileObj != null) {
                DbgLog.msg("ftc9773: RepetitiveActions: Closing navxDegrees fileobj");
                this.fileObj.close();
            }
        }
    }

    public RepetitiveActions(FTCRobot robot, LinearOpMode curOpMode, String loopRuntimeLog,
                             String rangeSensorLog, String navxLog) {
        this.robot = robot;
        this.curOpMode = curOpMode;
        this.loopRuntimeLog = loopRuntimeLog;
        this.rangeSensorLog = rangeSensorLog;
        this.navxLog = navxLog;
    }

    public void addAction(repActionBaseClass action) {
        this.repActions.add(action);
    }

    public void removeAction(repActionBaseClass action) {
        this.repActions.remove(action);
    }

    public void startActions() {
        for (repActionBaseClass a: this.repActions) {
            a.startAction();
        }
    }

    public void repeatActions() {
        for (repActionBaseClass a: this.repActions) {
            a.repeatAction();
        }
    }

    public void printToConsole() {
        for (repActionBaseClass a: this.repActions) {
            a.printToConsole();
        }
    }

    public void writeToFile() {
        for (repActionBaseClass a: this.repActions) {
            a.writeToFile();
        }
    }

    public void closeLog() {
        for (repActionBaseClass a: this.repActions) {
            a.closeLog();
        }
    }
}
