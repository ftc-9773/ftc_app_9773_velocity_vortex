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

    public LineFollow(FTCRobot robot, String lightSensorName, double lowSpeed,
                      double highSpeed) {
        this.driveSystem = robot.driveSystem;
        this.lightSensor = robot.curOpMode.hardwareMap.opticalDistanceSensor.get(lightSensorName);
        white = -1;
        black = -1;
        prevLight = -1;
    }

    public void searchForWhiteLine(){
        // ToDo:  Move the robot for ~ 2 seconds or until a white line is found.
        //  ToDo:  We may not actually need this if lineFollow can reliably find the white line
        return;
    }

    public void followLine() {
        light = lightSensor.getLightDetected();
        if (white == -1) {
            driveSystem.lineFollow(highSpeed, lowSpeed);
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
                driveSystem.lineFollow(highSpeed, lowSpeed);
            } else {
                driveSystem.lineFollow(highSpeed, highSpeed);
            }
        }

        DbgLog.msg(String.format("Light Detected= %f", light));
        DbgLog.msg(String.format("Threshold: %f", mid));
        //DbgLog.msg(String.format("MotorL power: %f", motor1.getPower()));
        //DbgLog.msg(String.format("MotorR power: %f", motor2.getPower()));
    }

}
