package org.firstinspires.ftc.teamcode.opmodes;

/*
 Created by Kids on 9/30/2016.
 */

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

//Luke edited Pranav's sample TeleOp on 9/23/2016

@TeleOp(name = "TestLineFollow", group = "TeleOp")

public class TestLineFollowOp extends LinearOpMode {

    DcMotor motor1 = null;
    DcMotor motor2 = null;
    OpticalDistanceSensor lightSensor = null;
    double prevLight = 0;
    double light = 0;
    double white = -1;
    double black = -1;
    double mid = -1;
    double lowSpeed = 0.05;
    double highSpeed = 0.15;

    public void runOpMode() throws InterruptedException{
        motor1 = hardwareMap.dcMotor.get("motor1");
        motor2 = hardwareMap.dcMotor.get("motor2");
        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        lightSensor = hardwareMap.opticalDistanceSensor.get("ods");

        waitForStart();
        long startTime = System.nanoTime();
        long elapsedTime = 0;
        while (opModeIsActive()){
            light = lightSensor.getLightDetected();
            if(white==-1){
                motor1.setPower(lowSpeed);
                motor2.setPower(highSpeed);
                if(light>prevLight)
                    prevLight = light;
                else{
                    white = prevLight;
                }
            }
            else if(black==-1){
                motor1.setPower(highSpeed);
                motor2.setPower(lowSpeed);
                if (light<prevLight)
                    prevLight=light;
                else if(light<white){
                    black = prevLight;
                }
            }
            else{
                mid = (black+white)/2;
                if(light>mid){
                    motor1.setPower(highSpeed);
                    motor2.setPower(lowSpeed);
                }
                else if(light<mid){
                    motor1.setPower(lowSpeed);
                    motor2.setPower(highSpeed);
                }
                else{
                    motor1.setPower(highSpeed);
                    motor2.setPower(highSpeed);
                }
            }

            DbgLog.msg(String.format("Light Detected= %f", light));
            DbgLog.msg(String.format("Threshold: %f", mid));
            //DbgLog.msg(String.format("MotorL power: %f", motor1.getPower()));
            //DbgLog.msg(String.format("MotorR power: %f", motor2.getPower()));

            elapsedTime = System.nanoTime()-startTime;

            if(elapsedTime/2500>= (long)1000000)
                break;

            idle();
        }
    }
}