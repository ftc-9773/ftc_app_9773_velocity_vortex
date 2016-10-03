/*
 * Copyright (c) 2016 Robocracy 9773.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.drivesys.Wheel;

import org.firstinspires.ftc.teamcode.drivesys.TwoMotorDrive;

@TeleOp(name = "TeleOpTest", group = "TeleOp")
public class TeleOpTest extends LinearOpMode{
    DcMotor motorL;
    DcMotor motorR;
    TwoMotorDrive drivesys;
    Wheel wheel;

    public void runOpMode() throws InterruptedException{
        motorL = hardwareMap.dcMotor.get("motorL");
        motorR = hardwareMap.dcMotor.get("motorR");
        wheel = new Wheel(Wheel.Type.RUBBER_TREADED);
        drivesys = new TwoMotorDrive(motorL, motorR, 1, 0, 1, 1, wheel);

        waitForStart();
        while (opModeIsActive()){
            float speed = gamepad1.left_stick_y;
            float direction = gamepad1.right_stick_x;

            drivesys.drive(speed, direction);
            idle();
        }
    }
}
