package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.ftccommon.DbgLog;

import org.json.JSONException;
import org.json.JSONObject;

/**
 * Created by ftcrobocracy on 10/14/16.
 */

public class AutonomousRecordReader extends JsonReader {
    public AutonomousRecordReader(String filePath) {
        super(filePath);
    }

    public DriveSysReader getDriveSysReader () {
        DriveSysReader driveSysReader = null;
        String driveSysFilePath;
        try {
            driveSysFilePath = this.jsonRoot.getString("robotConfigFilePath");
            driveSysReader = new DriveSysReader(driveSysFilePath);
        }
        catch (JSONException exc) {
            exc.printStackTrace();
        }
        return (driveSysReader);
    }

    public String getRecordFilePath() {
        String recordFilePath = null;
        String recordFilesDir = null;

        try {
            recordFilesDir = jsonRoot.getString("recordreplayFilesDir");
            recordFilePath = recordFilesDir + "/" + jsonRoot.getString("recordFileName");
        }
        catch (JSONException exc) {
            exc.printStackTrace();
        }
        return (recordFilePath);
    }
}
