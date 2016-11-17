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
    public JSONObject rangeObj=null;
    public JSONObject encoderVarsObj=null;

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
            key = JsonReader.getRealKeyIgnoreCase(navOptObj, "RangeSensor");
            if (key != null) {
                rangeObj = navOptObj.getJSONObject(key);
            }
            key = JsonReader.getRealKeyIgnoreCase(navOptObj, "DriveSysEncoderVariables");
            if (key != null) {
                encoderVarsObj = navOptObj.getJSONObject(key);
            }
        } catch (JSONException e) {
            e.printStackTrace();
        }
    }

    public boolean lineFollowerExists() {
        return (this.lfObj != null);
    }

    public boolean imuExists() {
        return (this.imuObj != null);
    }

    public boolean rangeSensorExists() { return (this.rangeObj != null); }

    public boolean encoderVarsExist() { return (this.encoderVarsObj != null); }

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

    public double getLFvariableDouble(String variableName) {
        double value = 0.0;
        JSONObject lfVarObj;
        try {
            String key = JsonReader.getRealKeyIgnoreCase(lfObj, "lineFollowVariables");
            lfVarObj = lfObj.getJSONObject(key);
            key = JsonReader.getRealKeyIgnoreCase(lfVarObj, variableName);
            value = lfVarObj.getDouble(key);
            DbgLog.msg("getLFvariableDouble(): key = %s, value=%f", key, value);
        } catch (JSONException e) {
            e.printStackTrace();
        }
        return (value);
    }

/*
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

    public double getLineFollowTimeOut() {
        double timeoutSec = 0.0;
        JSONObject lfVarObj;

        try {
            String key = JsonReader.getRealKeyIgnoreCase(lfObj, "lineFollowVariables");
            lfVarObj = lfObj.getJSONObject(key);
            key = JsonReader.getRealKeyIgnoreCase(lfVarObj, "timeOut");
            timeoutSec = lfVarObj.getDouble(key);
        } catch (JSONException e) {
            e.printStackTrace();
        }
        return (timeoutSec);
    }
*/

    public String getIMUDIMname() {
        String imuName = null;
        try {
            String key = JsonReader.getRealKeyIgnoreCase(imuObj, "DIMname");
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

    public double getIMUdriveSysInitialPower() {
        double driveSysPower=0.0;

        try {
            String key = JsonReader.getRealKeyIgnoreCase(imuObj, "driveSysInitialPower");
            driveSysPower = imuObj.getDouble(key);
        } catch (JSONException e) {
            e.printStackTrace();
        }
        return (driveSysPower);
    }

    public double getIMUdriveSysTargetPower() {
        double driveSysPower=0.0;

        try {
            String key = JsonReader.getRealKeyIgnoreCase(imuObj, "driveSysTargetPower");
            driveSysPower = imuObj.getDouble(key);
        } catch (JSONException e) {
            e.printStackTrace();
        }
        return (driveSysPower);
    }

    public double getIMUAngleTolerance() {
        double angleTolerance=0.0;
        try {
            String key = JsonReader.getRealKeyIgnoreCase(imuObj, "angleTolerance");
            angleTolerance = imuObj.getDouble(key);
        } catch (JSONException e) {
            e.printStackTrace();
        }
        return angleTolerance;
    }

    public double getTurningMaxSpeed() {
        double maxSpeed=1.0;
        try {
            String key = JsonReader.getRealKeyIgnoreCase(encoderVarsObj, "TurningMaxSpeed");
            maxSpeed = encoderVarsObj.getDouble(key);
        } catch (JSONException e) {
            e.printStackTrace();
        }
        return (maxSpeed);
    }

    public double getStraightLineMaxSpeed() {
        double maxSpeed=1.0;
        try {
            String key = JsonReader.getRealKeyIgnoreCase(encoderVarsObj, "StraightLineMaxSpeed");
            maxSpeed = encoderVarsObj.getDouble(key);
        } catch (JSONException e) {
            e.printStackTrace();
        }
        return (maxSpeed);
    }

    public double getLineFollowMaxSpeed() {
        double maxSpeed=1.0;
        try {
            String key = JsonReader.getRealKeyIgnoreCase(encoderVarsObj, "LineFollowMaxSpeed");
            maxSpeed = encoderVarsObj.getDouble(key);
        } catch (JSONException e) {
            e.printStackTrace();
        }
        return (maxSpeed);
    }
}
