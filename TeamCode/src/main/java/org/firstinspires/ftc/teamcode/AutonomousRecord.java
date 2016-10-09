package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drivesys.TwoMotorDrive;
import org.firstinspires.ftc.teamcode.drivesys.Wheel;
import org.firstinspires.ftc.teamcode.util.FileRW;

@Autonomous(name = "Record", group = "Record")
public class AutonomousRecord extends LinearOpMode {
    DcMotor motorL;
    DcMotor motorR;
    TwoMotorDrive drivesys;
    Wheel wheel;
    FileRW fileRW;

    @Override
    public void runOpMode() throws InterruptedException{
        motorL = hardwareMap.dcMotor.get("motorL");
        motorR = hardwareMap.dcMotor.get("motorR");
        wheel = new Wheel(Wheel.Type.RUBBER_TREADED);
        drivesys = new TwoMotorDrive(motorL, motorR, 1, 0, 1, 1, wheel);
        fileRW = new FileRW("/sdcard/FIRST/autonomous/autonomousPath.txt", true);

        waitForStart();
        //long startingTime = System.nanoTime();
        //long elapsedTime = 0;
        while (opModeIsActive()) {
            //long timestamp = System.nanoTime() - (startingTime + elapsedTime);
            double speed = gamepad1.left_stick_y * 0.5;
            double direction = gamepad1.right_stick_x * 0.5;

            fileRW.fileWrite(Double.toString(speed) + "," + Double.toString(direction));
            drivesys.drive((float) speed, (float) direction);

            DbgLog.msg(String.format("Speed: %f", speed, " , Direction: %f", direction));

            //elapsedTime = System.nanoTime() - startingTime;
            if(gamepad1.a){
                break;
            }
            sleep(5);
            idle();
        }
        DbgLog.msg("Is close executing?");
        fileRW.close();

    }
}
