package org.firstinspires.ftc.teamcode.recordreplay;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FTCRobot;
import org.firstinspires.ftc.teamcode.util.FileRW;

/**
 * Created by rsburugula on 11/26/16.
 */

public class SensorRecord extends RecordReplaySys {
    String filePath;
    String[] columns = null;
    FileRW fileRW;
    LinearOpMode curOpMode;
    FTCRobot robot;

    public SensorRecord(LinearOpMode curOpMode, FTCRobot robot, String filePath, String[] columns){
        this.curOpMode = curOpMode;
        this.robot = robot;
        this.filePath = filePath;
        this.fileRW = new FileRW(filePath, true);
    }
}
