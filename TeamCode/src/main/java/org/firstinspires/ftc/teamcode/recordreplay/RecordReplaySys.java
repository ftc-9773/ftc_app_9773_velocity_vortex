package org.firstinspires.ftc.teamcode.recordreplay;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FTCRobot;
import org.firstinspires.ftc.teamcode.util.FileRW;
import org.firstinspires.ftc.teamcode.util.JsonReaders.JsonReader;
import org.firstinspires.ftc.teamcode.util.JsonReaders.RecordReplaySysReader;

/**
 * Created by rsburugula on 11/26/16.
 */

public abstract class RecordReplaySys {
    LinearOpMode curOpMode;
    FTCRobot robot;
    String recordType;
    String filePath;
    FileRW fileRW;


    public RecordReplaySys(LinearOpMode curOpMode, FTCRobot robot, String recordType, String filePath, FileRW fileRW){
        this.curOpMode = curOpMode;
        this.robot = robot;
        this.filePath = filePath;
        this.recordType = recordType;
        this.fileRW = fileRW;
    }

    public RecordReplaySys(){}

    public static RecordReplaySys createRecordReplaySystem(LinearOpMode curOpMode, FTCRobot robot, String recordType,
                                                           String filePath){
        RecordReplaySys recordReplaySys = null;
        RecordReplaySysReader recordReplaySysReader = new RecordReplaySysReader(JsonReader.recordReplaySysFile, recordType);


        switch (recordType){
            case "sensor":
                String[] columns = recordReplaySysReader.getColumns();
                SensorRecord sensorRecord = new SensorRecord(curOpMode, robot, filePath, columns);

                sensorRecord.
        }

        return recordReplaySys;
    }
}
