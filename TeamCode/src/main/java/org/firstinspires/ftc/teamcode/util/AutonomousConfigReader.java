package org.firstinspires.ftc.teamcode.util;

import org.json.JSONException;

/**
 * Created by ftcrobocracy on 10/14/16.
 */

public class AutonomousConfigReader extends JsonReader {
    public AutonomousConfigReader(String filePath) {
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
}
