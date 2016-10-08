package org.firstinspires.ftc.teamcode;

/*
 Created by Kids on 9/30/2016.
 */

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

//Luke edited Pranav's sample TeleOp on 9/23/2016

@TeleOp(name = "TestLineFollow", group = "TeleOp")

public class TestLineFollow extends LinearOpMode{

    DcMotor motor1 = null;
    DcMotor motor2 = null;
    OpticalDistanceSensor lightSensor = null;
    double prevLight = 0;
    double light = 0;
    double white = -1;
    double black = -1;
    double mid = -1;

    public void runOpMode() throws InterruptedException{
        motor1 = hardwareMap.dcMotor.get("motor1");
        motor2 = hardwareMap.dcMotor.get("motor2");
        lightSensor = hardwareMap.opticalDistanceSensor.get("ods");

        waitForStart();
        while (opModeIsActive()){
            light = lightSensor.getLightDetected();
            if(white==-1){
                motor1.setPower(0.9);
                motor2.setPower(1);
                if(light>prevLight)
                    prevLight = light;
                else{
                    white = prevLight;
                }
            }
            else if(black==-1){
                motor1.setPower(1);
                motor2.setPower(0.9);
                if (light<prevLight)
                    prevLight=light;
                else if(light<white){
                    black = prevLight;
                }
            }
            else{
                mid = (black+white)/2;
                if(light>mid){
                    motor1.setPower(1);
                    motor2.setPower(0.9);
                }
                else if(light<mid){
                    motor1.setPower(0.9);
                    motor2.setPower(1);
                }
                else{
                    motor1.setPower(1);
                    motor2.setPower(1);
                }
            }

            idle();
        }
    }
}