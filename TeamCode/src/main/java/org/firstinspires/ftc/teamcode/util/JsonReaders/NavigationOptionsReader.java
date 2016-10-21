package org.firstinspires.ftc.teamcode.util.JsonReaders;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;


public class NavigationOptionsReader extends JsonReader {
    JSONObject lfObj;
    JSONObject imuObj;
    public NavigationOptionsReader(String filePath) {

        super(filePath);
        try {
            String key = JsonReader.getRealKeyIgnoreCase(jsonRoot, "LineFollow");
            lfObj = jsonRoot.getJSONObject(key);
            key = JsonReader.getRealKeyIgnoreCase(jsonRoot, "imu");
            imuObj = jsonRoot.getJSONObject(key);
        } catch (JSONException e) {
            e.printStackTrace();
        }
    }

    public String getLightSensorName() {
        String lightSensorName = null;
        JSONObject lightSensorObj;
        try {
            String key = JsonReader.getRealKeyIgnoreCase(lfObj, "LightSensor");
            lightSensorObj = lfObj.getJSONObject(key);
            key = JsonReader.getRealKeyIgnoreCase(lfObj, "name");
            lightSensorName = lfObj.getString(key);
        } catch (JSONException e) {
            e.printStackTrace();
        }
        return (lightSensorName);
    }

    public String getLightSensorType() {
        String sensorType = null;
        JSONObject lightSensorObj;
        try {
            String key = JsonReader.getRealKeyIgnoreCase(lfObj, "LightSensor");
            lightSensorObj = lfObj.getJSONObject(key);
            key = JsonReader.getRealKeyIgnoreCase(lfObj, "type");
            sensorType = lfObj.getString(key);
        } catch (JSONException e) {
            e.printStackTrace();
        }
        return (sensorType);
    }

    public double  getLineFollowLowSpeed() {
        double minSpeed = 0.0;
        // ToDo
        return (minSpeed);
    }

    public double  getLineFollowHighSpeed() {
        double maxSpeed = 0.0;
        // ToDo
        return (maxSpeed);
    }

    public String getIMUname() {
        String imuName = null;
        try {
            String key = JsonReader.getRealKeyIgnoreCase(imuObj, "name");
            imuName = imuObj.getString(key);
        } catch (JSONException e) {
            e.printStackTrace();
        }
        return (imuName);
    }

    public int getIMUportNum() {
        int imuPortNum = 0;
        try {
            String key = JsonReader.getRealKeyIgnoreCase(imuObj, "portnum");
            imuPortNum = imuObj.getInt(key);
        } catch (JSONException e) {
            e.printStackTrace();
        }
        return (imuPortNum);
    }

}
