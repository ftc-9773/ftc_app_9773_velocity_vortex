package org.firstinspires.ftc.teamcode.navigation;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.FTCRobot;
import org.firstinspires.ftc.teamcode.drivesys.DriveSystem;

/**
 * Created by Luke on 10/15/2016.
 */

/*
 * Copyright (c) 2016 Robocracy 9773
 */

public class LineFollow{

    OpticalDistanceSensor lightSensor;
    double white, black, mid;
    double lowSpeed, highSpeed;
    double odsOffset;
    double prevLight;
    double basePower, Kp;
    DriveSystem driveSystem;
    long stopTimeStamp=0;
    long timoutNanoSec=0;
    FTCRobot robot;

    public LineFollow(FTCRobot robot, String lightSensorName, double lowSpeed,
                      double highSpeed, double lineFollowTimeOut,
                      double white, double black) {
        this.robot = robot;
        this.driveSystem = robot.driveSystem;
        this.lightSensor = robot.curOpMode.hardwareMap.opticalDistanceSensor.get(lightSensorName);
        this.lowSpeed = lowSpeed;
        this.highSpeed = highSpeed;
        this.basePower = (lowSpeed+highSpeed)/2;
        this.white = white;
        this.black = black;
        this.mid = (white + black) / 2;
        //this.Kp = (highSpeed-this.basePower) / (white - this.mid);
        this.Kp = 0.5;
        this.odsOffset = robot.distanceLeft / (robot.distanceLeft + robot.distanceRight);
        this.timoutNanoSec = (long) (lineFollowTimeOut * 1000000000L);
        DbgLog.msg("ftc9773: sensorName=%s, lowSpeed=%f, highSpeed=%f, timeoutNanoSec=%d",
                lightSensorName, lowSpeed, highSpeed, this.timoutNanoSec);
        DbgLog.msg("ftc9773: Kp = %f, odsOffset=%f", this.Kp, this.odsOffset);
//        this.white = -1;
//        this.black = -1;
        prevLight = -1;
    }

    public void searchForWhiteLine(){
        // ToDo:  Move the robot for ~ 2 seconds or until a white line is found.
        //  ToDo:  We may not actually need this if turnOrSpin can reliably find the white line
        return;
    }

    public void turnUntilWhiteLine(boolean spinClockwise) {
        double leftInitialPower=0.0, rightInitialPower=0.0;
        driveSystem.setMaxSpeed((float) this.robot.navigation.turnMaxSpeed);
        if(spinClockwise){
            leftInitialPower = 0.3;
            rightInitialPower = -leftInitialPower;
        }
        else{
            leftInitialPower = -0.3;
            rightInitialPower = -leftInitialPower;
        }
        while ((lightSensor.getLightDetected()<this.mid) && robot.curOpMode.opModeIsActive()) {
            driveSystem.turnOrSpin(leftInitialPower,rightInitialPower);
//            if (lightSensor.getLightDetected()<this.mid)
//                break;
        }
        driveSystem.stop();
        driveSystem.resumeMaxSpeed();

    }
    public void driveUntilWhiteLine(double speed, long timeoutMillis){
        // timeout puts an upper limit on how long the while loop can run
        long startTime = System.nanoTime();
        this.robot.driveSystem.setMaxSpeed((float) speed);
        while((lightSensor.getLightDetected()<this.mid) && robot.curOpMode.opModeIsActive()
                && ((System.nanoTime() - startTime) < (timeoutMillis*1000000))) {
            driveSystem.drive((float) 1.0, 0);
            DbgLog.msg("ftc9773: light detected = %f", lightSensor.getLightDetected());
        }
        DbgLog.msg("ftc9773: light detected = %f", lightSensor.getLightDetected());
        driveSystem.stop();
        driveSystem.resumeMaxSpeed();
    }

    public void drivePastWhiteLine(double speed) {
        // timeout puts an upper limit on how long the while loop can run
        ElapsedTime timeout = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        this.robot.driveSystem.setMaxSpeed((float) speed);
        timeout.reset();
        while((lightSensor.getLightDetected()>this.mid) && robot.curOpMode.opModeIsActive()
                && (timeout.milliseconds() < 1000)) {
            driveSystem.drive((float) 1.0, 0);
        }
        driveSystem.stop();
        driveSystem.resumeMaxSpeed();
    }

    public void printMinMaxLightDetected() {
        double minLight = 1.0, maxLight=0.0, curLight;
        driveSystem.setMaxSpeed((float)robot.navigation.turnMaxSpeed);
        robot.driveSystem.turnOrSpin(-0.4, 0.4);
        double initialYaw = robot.navigation.gyro.getYaw();
        double diffYaw=0.0;
        while ((diffYaw < 45.0) && robot.curOpMode.opModeIsActive()) {
            curLight = robot.navigation.lf.lightSensor.getLightDetected();
            if (minLight > curLight) minLight = curLight;
            if (maxLight < curLight) maxLight = curLight;
            diffYaw = Math.abs(robot.navigation.gyro.getYaw() - initialYaw);
            DbgLog.msg("ftc9773: diffYaw=%f, minLight=%f, maxLight=%f, curLight=%f", diffYaw,
                    minLight, maxLight, curLight);
        }
        driveSystem.stop();
        driveSystem.resumeMaxSpeed();
        robot.curOpMode.telemetry.addData("Light Detected:", "minLight=%f, maxLight=%f", minLight, maxLight);
        robot.curOpMode.telemetry.update();
    }

    public void followLineProportional() {
        double light = lightSensor.getLightDetected();

//        double lightOffset = (light-mid)/(white-mid);
//        double odsOffset = (this.robot.distanceLeft/(this.robot.distanceLeft+this.robot.distanceRight));
//        double leftPower = basePower+ Kp*lightOffset;
//        double rightPower = basePower- Kp*lightOffset;
//        leftPower = leftPower*odsOffset*2;
//        rightPower = rightPower*(1-odsOffset)*2;

        double error = mid - light;
        double correction = this.Kp * error;
        double leftCorrection = this.odsOffset * correction;
        double rightCorrection = (1 - this.odsOffset) * correction;
        double leftPower = Range.clip(basePower - leftCorrection, -1.0, 1.0);
        double rightPower = Range.clip(basePower + rightCorrection, -1.0, 1.0);
        DbgLog.msg("ftc9773: lightDetected = %f, error = %f, correction = %f, left correction = %f, right correction = %f, leftPower=%f, rightPower=%f",
                light, error, correction, leftCorrection, rightCorrection, leftPower,rightPower);
        driveSystem.turnOrSpin(leftPower, rightPower);

//        DbgLog.msg("ftc9773: lightDetected = %f", lightSensor.getLightDetected());
//        DbgLog.msg("ftc9773: error=%f, correction=%f, leftPower=%f, rightPower=%f",
//                error, correction, leftPower, rightPower);
    }

    public boolean onWhiteLine() {
        return (lightSensor.getLightDetected() >= this.mid);
    }
}
