package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/**
 * Created by Kids on 10/15/2016.
 */
public class LineFollow{

    DcMotor motorL;
    DcMotor motorR;
    OpticalDistanceSensor lightSensor;
    double white;
    double black;
    double lowSpeed;
    double highSpeed;
    double light;
    double prevLight;
    double mid;

    public LineFollow(DcMotor motor1, DcMotor motor2, OpticalDistanceSensor ods){
        motorL = motor1;
        motorR = motor2;
        lightSensor = ods;
        white = -1;
        black = -1;
        prevLight = -1;
    }

    public void searchForWhiteLine(){

    }

    public void followLine() {
        light = lightSensor.getLightDetected();
        if (white == -1) {
            motorL.setPower(lowSpeed);
            motorR.setPower(highSpeed);
            if (light > prevLight)
                prevLight = light;
            else {
                white = prevLight;
            }
        } else if (black == -1) {
            motorL.setPower(highSpeed);
            motorR.setPower(lowSpeed);
            if (light < prevLight)
                prevLight = light;
            else if (light < white) {
                black = prevLight;
            }
        } else {
            mid = (black + white) / 2;
            if (light > mid) {
                motorL.setPower(highSpeed);
                motorR.setPower(lowSpeed);
            } else if (light < mid) {
                motorL.setPower(lowSpeed);
                motorR.setPower(highSpeed);
            } else {
                motorL.setPower(highSpeed);
                motorR.setPower(highSpeed);
            }
        }

        DbgLog.msg(String.format("Light Detected= %f", light));
        DbgLog.msg(String.format("Threshold: %f", mid));
        //DbgLog.msg(String.format("MotorL power: %f", motor1.getPower()));
        //DbgLog.msg(String.format("MotorR power: %f", motor2.getPower()));
    }

}
