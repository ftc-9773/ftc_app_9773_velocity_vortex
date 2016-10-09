package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drivesys.TwoMotorDrive;
import org.firstinspires.ftc.teamcode.drivesys.Wheel;
import org.firstinspires.ftc.teamcode.util.FileRW;

@Autonomous(name = "Replay", group = "Autonomous")
public class AutonomousReplay extends LinearOpMode {
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
        fileRW = new FileRW("/sdcard/FIRST/autonomous/autonomousPath.txt", false);

        waitForStart();
        while (opModeIsActive()){
            String line = fileRW.getNextLine();
            if(line != null) {
                String[] lineElements = line.split(",");
                if(lineElements.length < 2){
                    continue;
                }
                else {
                    //long timestamp = Long.parseLong(lineElements[0]);
                    double speed = Double.parseDouble(lineElements[0]);
                    double direction = Double.parseDouble(lineElements[1]);

                    //sleep(timestamp);
                    drivesys.drive((float) speed, (float) direction);
                }
            }
            else{
                stop();
            }
            sleep(5);
        }
        fileRW.close();
    }
}
