package org.firstinspires.ftc.teamcode.navigation;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.teamcode.FTCRobot;
import org.firstinspires.ftc.teamcode.drivesys.DriveSystem;

/**
 * Created by Kids on 10/15/2016.
 */
public class LineFollow{

    OpticalDistanceSensor lightSensor;
    double white;
    double black;
    double lowSpeed;
    double highSpeed;
    double light;
    double prevLight;
    double mid;
    DriveSystem driveSystem;
    long stopTimeStamp=0;
    long timoutNanoSec=0;

    public LineFollow(FTCRobot robot, String lightSensorName, double lowSpeed,
                      double highSpeed, double lineFollowTimeOut) {
        this.driveSystem = robot.driveSystem;
        this.lightSensor = robot.curOpMode.hardwareMap.opticalDistanceSensor.get(lightSensorName);
        this.lowSpeed = lowSpeed;
        this.highSpeed = highSpeed;
        this.timoutNanoSec = (long) (lineFollowTimeOut * 1000000000L);
        DbgLog.msg("sensorName=%s, lowSpeed=%f, highSpeed=%f, timeoutNanoSec=%d",
                lightSensorName, lowSpeed, highSpeed, this.timoutNanoSec);
        white = -1;
        black = -1;
        prevLight = -1;
    }

    public void searchForWhiteLine(){
        // ToDo:  Move the robot for ~ 2 seconds or until a white line is found.
        //  ToDo:  We may not actually need this if lineFollow can reliably find the white line
        return;
    }

    public void setStartTimeStamp() {
        this.stopTimeStamp = System.nanoTime() + this.timoutNanoSec;
    }

    public boolean timeoutReached() {
        return (System.nanoTime() >= this.stopTimeStamp);
    }

    public void followLine() {
        light = lightSensor.getLightDetected();
        if (white == -1) {
            driveSystem.lineFollow(lowSpeed, highSpeed);
            if (light > prevLight)
                prevLight = light;
            else {
                white = prevLight;
            }
        } else if (black == -1) {
            driveSystem.lineFollow(highSpeed, lowSpeed);
            if (light < prevLight)
                prevLight = light;
            else if (light < white) {
                black = prevLight;
            }
        } else {
            mid = (black + white) / 2;
            if (light > mid) {
                driveSystem.lineFollow(highSpeed, lowSpeed);
            } else if (light < mid) {
                driveSystem.lineFollow(lowSpeed, highSpeed);
            } else {
                driveSystem.lineFollow(highSpeed, highSpeed);
            }
        }

        DbgLog.msg(String.format("Light Detected= %f, mid=%f", light, mid));
        //DbgLog.msg(String.format("MotorL power: %f", motor1.getPower()));
        //DbgLog.msg(String.format("MotorR power: %f", motor2.getPower()));
    }

}
