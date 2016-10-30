/*
 * Copyright (c) 2016 Robocracy 9773.
 */

package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drivesys.TwoMotorDrive;
import org.firstinspires.ftc.teamcode.drivesys.Wheel;

@TeleOp(name = "TeleOp2MotorTank", group = "TeleOp")
public class TeleOp2MotorTank extends LinearOpMode {
    DcMotor motorL;
    DcMotor motorR;
    TwoMotorDrive drivesys;
    Wheel wheel;

    @Override
    public void runOpMode() throws InterruptedException{
        motorL = hardwareMap.dcMotor.get("motorL");
        motorR = hardwareMap.dcMotor.get("motorR");
        wheel = new Wheel("rubber-treaded", 4.0);
        drivesys = new TwoMotorDrive(motorL, motorR, 1, 0, 1, wheel, 1120);

        waitForStart();
        while (opModeIsActive()){
            float speed = gamepad1.left_stick_y;
            float direction = gamepad1.right_stick_x;

            drivesys.drive(speed, direction);
            idle();
        }
    }
}
