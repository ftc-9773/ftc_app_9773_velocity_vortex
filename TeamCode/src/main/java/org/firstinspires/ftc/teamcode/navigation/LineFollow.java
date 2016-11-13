package org.firstinspires.ftc.teamcode.navigation;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.teamcode.FTCRobot;
import org.firstinspires.ftc.teamcode.drivesys.DriveSystem;

/**
 * Created by Luke on 10/15/2016.
 */
public class LineFollow{

    OpticalDistanceSensor lightSensor;
    double white, black, mid;
    double lowSpeed, highSpeed;
    double light;
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
        this.Kp = (highSpeed-lowSpeed)/2;
        this.white = white;
        this.black = black;
        this.mid = (white + black) / 2;
        this.timoutNanoSec = (long) (lineFollowTimeOut * 1000000000L);
        DbgLog.msg("sensorName=%s, lowSpeed=%f, highSpeed=%f, timeoutNanoSec=%d",
                lightSensorName, lowSpeed, highSpeed, this.timoutNanoSec);
//        this.white = -1;
//        this.black = -1;
        prevLight = -1;
    }

    public void searchForWhiteLine(){
        // ToDo:  Move the robot for ~ 2 seconds or until a white line is found.
        //  ToDo:  We may not actually need this if turnOrSpin can reliably find the white line
        return;
    }

    public void setStartTimeStamp() {
        this.stopTimeStamp = System.nanoTime() + this.timoutNanoSec;
        driveSystem.turnOrSpin(this.highSpeed,this.highSpeed);
    }

    public boolean timeoutReached() {
        return (System.nanoTime() >= this.stopTimeStamp);
    }

    public void followLine() {
        light = lightSensor.getLightDetected();
        if (white == -1) {
            driveSystem.turnOrSpin(lowSpeed, highSpeed);
            if (light > prevLight)
                prevLight = light;
            else {
                white = prevLight;
            }
        } else if (black == -1) {
            driveSystem.turnOrSpin(highSpeed, lowSpeed);
            if (light < prevLight)
                prevLight = light;
            else if (light < white) {
                black = prevLight;
            }
        } else {
            mid = (black + white) / 2;
            if (light > mid) {
                driveSystem.turnOrSpin(highSpeed, lowSpeed);
            } else if (light < mid) {
                driveSystem.turnOrSpin(lowSpeed, highSpeed);
            } else {
                driveSystem.turnOrSpin(highSpeed, highSpeed);
            }
        }

        DbgLog.msg(String.format("Light Detected= %f, mid=%f", light, mid));
        DbgLog.msg("ahtoiqhtoiwnegt    o");
        //DbgLog.msg(String.format("MotorL power: %f", motor1.getPower()));
        //DbgLog.msg(String.format("MotorR power: %f", motor2.getPower()));
    }

    public void turnUntilWhiteLine(boolean spinClockwise) {
        double leftInitialPower=0.4, rightInitialPower=0.0;
        this.robot.driveSystem.setMaxSpeed((float) 0.4);
        if(spinClockwise){
            leftInitialPower = 0.4;
            rightInitialPower = -leftInitialPower;
        }
        else{
            leftInitialPower = -0.4;
            rightInitialPower = -leftInitialPower;
        }
        while (true) {
            this.robot.driveSystem.turnOrSpin(leftInitialPower,rightInitialPower);
            if (lightSensor.getLightDetected()>=this.mid)
                break;
        }
        this.robot.driveSystem.stop();

    }


    public void followLineProportional() {
        light = lightSensor.getLightDetected();

        double lightOffset = (light-mid)/(white-mid);
        double leftPower = basePower+ Kp*lightOffset;
        double rightPower = basePower- Kp*lightOffset;
        driveSystem.turnOrSpin(leftPower, rightPower);

        DbgLog.msg("Here! light = %f, light offset = %f, leftPower=%f, rightPower=%f", light, lightOffset,leftPower,rightPower);

//        DbgLog.msg("lightDetected = %f", lightSensor.getLightDetected());
//        DbgLog.msg("error=%f, correction=%f, leftPower=%f, rightPower=%f",
//                error, correction, leftPower, rightPower);
    }

}
