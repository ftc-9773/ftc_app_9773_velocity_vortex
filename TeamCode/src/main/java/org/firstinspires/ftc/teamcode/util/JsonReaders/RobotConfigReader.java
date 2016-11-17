package org.firstinspires.ftc.teamcode.util.JsonReaders;

import com.google.gson.JsonObject;
import com.qualcomm.ftccommon.DbgLog;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.util.List;

/**
 * Created by pranavb on 10/15/16.
 */

public class RobotConfigReader extends JsonReader {
    String robotName;
    JSONObject robotObj=null;
    public RobotConfigReader(String filePath, String robotName)
    {
        super(filePath);
        this.robotName = robotName;
        try {
            this.robotObj = jsonRoot.getJSONObject(robotName);
        } catch (JSONException e) {
            e.printStackTrace();
        }
    }

    public String getDriveSysName(){
        String name = null;
        try{
            String key = JsonReader.getRealKeyIgnoreCase(robotObj, "driveSystem");
            name = robotObj.getString(key);
        }catch (JSONException e){
            e.printStackTrace();
        }
        return name;
    }

    public String getNavigationOption() {
        // ToDo
        String navigationOption = null;
        try{
            String key = JsonReader.getRealKeyIgnoreCase(robotObj, "navigation");
            navigationOption = robotObj.getString(key);
        }catch (JSONException e){
            e.printStackTrace();
            DbgLog.error("navigation key not found for the robot named %s!", robotName);
        }
        return (navigationOption);
    }

    public String[] getAttachments() {
        int len = 0;
        String[] attachmentsArr = null;
        try {
            JSONArray attachs = robotObj.getJSONArray("attachments");
            len = attachs.length();
            DbgLog.msg("Length of attachs array = %d", attachs.length());
            attachmentsArr = new String[len];
            for (int i = 0; i < len; i++) {
                attachmentsArr[i] = attachs.getString(i);
            }
        } catch (JSONException e) {
            e.printStackTrace();
            DbgLog.error("Problem finding one or more attachments for the robot named %s",
                    robotName);
        }
        return (attachmentsArr);
    }
}
