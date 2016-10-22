package org.firstinspires.ftc.teamcode.util.JsonReaders;

import com.qualcomm.ftccommon.DbgLog;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;


public class NavigationOptionsReader extends JsonReader {
    String navOptStr;
    public JSONObject navOptObj;
    public JSONObject lfObj=null;
    public JSONObject imuObj=null;

    public NavigationOptionsReader(String filePath, String navOptStr) {
        super(filePath);
        try {
            String key = JsonReader.getRealKeyIgnoreCase(jsonRoot, navOptStr);
            this.navOptObj = jsonRoot.getJSONObject(key);
            key = JsonReader.getRealKeyIgnoreCase(navOptObj, "LineFollow");
            if (key != null) {
                lfObj = navOptObj.getJSONObject(key);
            }
            key = JsonReader.getRealKeyIgnoreCase(navOptObj, "IMU");
            if (key != null) {
                imuObj = navOptObj.getJSONObject(key);
            }
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
            key = JsonReader.getRealKeyIgnoreCase(lightSensorObj, "name");
            lightSensorName = lightSensorObj.getString(key);
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
            key = JsonReader.getRealKeyIgnoreCase(lightSensorObj, "type");
            sensorType = lightSensorObj.getString(key);
        } catch (JSONException e) {
            e.printStackTrace();
        }
        return (sensorType);
    }

    public double  getLineFollowLowSpeed() {
        JSONObject lfVarObj;
        double lowSpeed = 0.0;
        try {
            String key = JsonReader.getRealKeyIgnoreCase(lfObj, "lineFollowVariables");
            lfVarObj = lfObj.getJSONObject(key);
            key = JsonReader.getRealKeyIgnoreCase(lfVarObj, "lowSpeed");
            lowSpeed = lfVarObj.getDouble(key);
        } catch (JSONException e) {
            e.printStackTrace();
        }
        return (lowSpeed);
    }

    public double  getLineFollowHighSpeed() {
        JSONObject lfVarObj;
        double highSpeed = 0.0;
        try {
            String key = JsonReader.getRealKeyIgnoreCase(lfObj, "lineFollowVariables");
            lfVarObj = lfObj.getJSONObject(key);
            key = JsonReader.getRealKeyIgnoreCase(lfVarObj, "highSpeed");
            highSpeed = lfVarObj.getDouble(key);
        } catch (JSONException e) {
            e.printStackTrace();
        }
        return (highSpeed);
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
