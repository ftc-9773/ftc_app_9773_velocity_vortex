package org.firstinspires.ftc.teamcode.util.JsonReaders;

import org.json.JSONException;
import org.json.JSONObject;

/**
 * Created by pranavb on 10/15/16.
 */

public class DriveSysReader extends JsonReader {
    JSONObject driveSysObj;
    String driveSysType;

    public DriveSysReader(String filePath, String driveSysType)
    {
        super(filePath);
        try {
            driveSysType = JsonReader.getRealKeyIgnoreCase(jsonRoot, driveSysType);
            driveSysObj = jsonRoot.getJSONObject(driveSysType);
            this.driveSysType = driveSysType;
        }catch (JSONException e){
            e.printStackTrace();
        }
    }

    public String getMotorType(String motorName) {
        String motorType = null;
        JSONObject obj;

        try {
            String key = JsonReader.getRealKeyIgnoreCase(driveSysObj, "Motors");
            obj = driveSysObj.getJSONObject(key);
            key = JsonReader.getRealKeyIgnoreCase(obj, motorName);
            motorType = obj.getString(key);
        } catch (JSONException e) {
            e.printStackTrace();
        }
        return (motorType);
    }

    public String getWheelType(){
        JSONObject obj = null;
        String wheelType = new String();
        try {
            String key = JsonReader.getRealKeyIgnoreCase(driveSysObj, "WheelType");
            wheelType = jsonRoot.getString(key);
        } catch (JSONException e){
            e.printStackTrace();
        }

        return wheelType;
    }

    public String getDriveSysType() {
        return driveSysType;
    }
}
