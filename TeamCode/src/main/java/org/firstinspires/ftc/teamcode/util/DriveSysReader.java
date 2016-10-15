package org.firstinspires.ftc.teamcode.util;

import com.google.gson.JsonParseException;
import com.google.gson.JsonParser;
import com.google.gson.internal.ObjectConstructor;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.util.ArrayList;

/**
 * Created by pranavb on 10/15/16.
 */

public class DriveSysReader extends JsonReader {
    public DriveSysReader(String filePath) {
        super(filePath);
    }

    public String getDriveSysType(){
        String name = new String();
        try{
            name = jsonRoot.getString("Name");
        }catch (JSONException e){
            e.printStackTrace();
        }

        return name;
    }

    public JSONObject getMotors(){
        JSONObject obj = new JSONObject();
        try {
            obj = jsonRoot.getJSONObject("Motors");
        }catch (JSONException e){
            e.printStackTrace();
        }

        return obj;
    }

    public String getWheelType(){
        String wheelType = new String();
        try {
            wheelType = jsonRoot.getString("WheelType");
        } catch (JSONException e){
            e.printStackTrace();
        }

        return wheelType;
    }

    public double getWheelDiameter(){
        double wheelDiameter = 0;
        try {
            wheelDiameter = jsonRoot.getDouble("WheelDiameter");
        } catch (JSONException e){
            e.printStackTrace();
        }

        return wheelDiameter;
    }
}
