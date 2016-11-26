package org.firstinspires.ftc.teamcode.recordreplay;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FTCRobot;
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


    public RecordReplaySys(LinearOpMode curOpMode, FTCRobot robot, String recordType, String filePath){
        this.curOpMode = curOpMode;
        this.robot = robot;
        this.filePath = filePath;
        this.recordType = recordType;
    }

    public static RecordReplaySys createRecordReplaySystem(LinearOpMode curOpMode, FTCRobot robot, String recordType,
                                                           String filePath){
        RecordReplaySys recordReplaySys = null;
        RecordReplaySysReader recordReplaySysReader = new RecordReplaySysReader(JsonReader.recordReplaySysFile, recordType);

        switch (recordType){

        }

        return recordReplaySys;
    }
}
