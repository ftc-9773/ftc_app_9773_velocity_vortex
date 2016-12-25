package org.firstinspires.ftc.teamcode.util.JsonReaders;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;


/*
 * Copyright (c) 2016 Robocracy 9773
 */

public class AutonomousOptionsReader extends JsonReader {
    public JSONArray actions;
    public AutonomousOptionsReader(String filePath, String autonomousOption) {

        super(filePath);
        try {
            String key = JsonReader.getRealKeyIgnoreCase(jsonRoot, autonomousOption);
            actions = jsonRoot.getJSONArray(key);
        } catch (JSONException e) {
            e.printStackTrace();
        }
    }

    public JSONObject getAction(int actionNum){
        JSONObject jsonObject = null;
        try {
            jsonObject = (JSONObject)actions.get(actionNum);

        } catch (JSONException e) {
            e.printStackTrace();
        }
        return jsonObject;
    }

    public String getActionType(int actionNum) {
        String actionType = null;
        JSONObject obj = null;

        try {
            obj = (JSONObject) actions.get(actionNum);
            String key = JsonReader.getRealKeyIgnoreCase(obj, "type");
            actionType = obj.getString(key);
        } catch (JSONException e) {
            e.printStackTrace();
        }
        return (actionType);
    }

    public String getReplayFile(String allianceColor, int actionNum) {
        JSONObject obj = null;
        String replayFile = null;

        try {
            obj = (JSONObject) actions.get(actionNum);
            String key = JsonReader.getRealKeyIgnoreCase(obj, "value");
            replayFile = obj.getString(key);
            if (allianceColor.equalsIgnoreCase("red"))
                replayFile = JsonReader.autonomousRedDir + "/" + replayFile;
            else
                replayFile = JsonReader.autonomousBlueDir + "/" + replayFile;
        } catch (JSONException e) {
            e.printStackTrace();
        }
        return (replayFile);
    }

    public String getMethodName(int actionNum) {
        JSONObject obj = null;
        String methodName = null;

        try {
            obj = (JSONObject) actions.get(actionNum);
            String key = JsonReader.getRealKeyIgnoreCase(obj, "method");
            methodName = obj.getString(key);
        } catch (JSONException e) {
            e.printStackTrace();
        }
        return (methodName);
    }
}
