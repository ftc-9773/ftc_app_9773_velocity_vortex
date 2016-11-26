package org.firstinspires.ftc.teamcode.util.JsonReaders;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

/**
 * Created by rsburugula on 11/26/16.
 */

public class RecordReplaySysReader extends JsonReader {
    JSONObject recordReplaySysObject;
    String recordType;
    String filePath;

    public RecordReplaySysReader(String jsonFilePath, String recordType){
        super(jsonFilePath);
        try {
            recordType = JsonReader.getRealKeyIgnoreCase(jsonRoot, recordType);
            recordReplaySysObject = jsonRoot.getJSONObject(recordType);
            this.recordType = recordType;
        }
        catch (JSONException e){
            e.printStackTrace();
        }
    }

    public String getRecordType(){return recordType;}

    public String[] getColumns(){
        String[] columnsArray = null;
        try {
            JSONArray columns = recordReplaySysObject.getJSONArray("columns");
            columnsArray = new String[columns.length()];
            for (int i=0;i<columns.length();i++){
                columnsArray[i] = columns.getString(i);
            }
        }
        catch (JSONException e){
            e.printStackTrace();
        }
        return columnsArray;
    }

    public String getHandler(){
        String handler = null;
        try {
            handler = recordReplaySysObject.getString("handler");
        }
        catch (JSONException e){
            e.printStackTrace();
        }
        return handler;
    }

    public int getClockCycle(){
        int clockCycle = 0;
        try {
            clockCycle = recordReplaySysObject.getInt("clockCycle");
        }
        catch (JSONException e){
            e.printStackTrace();
        }
        return clockCycle;
    }
}
