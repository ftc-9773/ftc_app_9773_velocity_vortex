/*
 * Copyright (c) 2016 Robocracy 9773.
 */

package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drivesys.FourMotorTankDrive;
import org.firstinspires.ftc.teamcode.drivesys.Wheel;
import org.firstinspires.ftc.teamcode.util.JsonReaders.DriveSysReader;

@TeleOp(name = "TeleOp4MotorTank", group = "TeleOp")
public class TeleOp4MotorTank extends LinearOpMode {
    DcMotor motorL1;
    DcMotor motorL2;
    DcMotor motorR1;
    DcMotor motorR2;
    FourMotorTankDrive drivesys;
    Wheel wheel;

    @Override
    public void runOpMode() throws InterruptedException{

        motorL1 = hardwareMap.dcMotor.get("motorL1");
        motorL2 = hardwareMap.dcMotor.get("motorL2");
        motorR1 = hardwareMap.dcMotor.get("motorR1");
        motorR2 = hardwareMap.dcMotor.get("motorR2");
        wheel = new Wheel("rubber-treaded", 4.0);
        drivesys = new FourMotorTankDrive(motorL1, motorL2, motorR1, motorR2, 1, 0, 1, wheel, 1120);

        waitForStart();
        while (opModeIsActive()){
            float speed = gamepad1.left_stick_y;
            float direction = gamepad1.right_stick_x;

            drivesys.drive(speed, direction);
            idle();
        }
    }
}
