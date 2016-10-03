package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by michaelzhou on 9/30/16.
 */
@Autonomous(name = "Go Straight", group = "Autonomous")
public class GoStraightOpMode extends LinearOpMode{
    DcMotor leftMotor, rightMotor;
    double driveSpeed = 0.5;
    double totalRotations = (12*5 / 4*Math.PI)*1120;
    public void runOpMode() throws InterruptedException{
        leftMotor = hardwareMap.dcMotor.get("MotorL");
        rightMotor = hardwareMap.dcMotor.get("MotorR");

        waitForStart();

        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        leftMotor.setTargetPosition((int)totalRotations);
        rightMotor.setTargetPosition((int)totalRotations);
        leftMotor.setPower(driveSpeed);
        rightMotor.setPower(driveSpeed);

//        while(opModeIsActive()) {
//            if(leftMotor.getCurrentPosition()==counts||rightMotor.getCurrentPosition()==counts){
//
//            }
//        }
        idle();
    }
}