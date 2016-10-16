package org.firstinspires.ftc.teamcode.util;

import com.google.gson.JsonObject;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

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

    public JSONObject getAction(int actionNum){
        JSONObject jsonObject = null;
        try {
            JSONArray actions = this.jsonRoot.getJSONArray("autonomousActions");
            jsonObject = (JSONObject)actions.get(actionNum);

        } catch (JSONException e) {
            e.printStackTrace();
        }
        return jsonObject;
    }
}
