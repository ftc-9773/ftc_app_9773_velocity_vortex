package org.firstinspires.ftc.teamcode.util.JsonReaders;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

/**
 * Created by pranavb on 10/15/16.
 */

public class RobotConfigReader extends JsonReader {
    String robotName;
    public RobotConfigReader(String filePath, String robotName)
    {
        super(filePath);
        this.robotName = robotName;
    }

    public String getDriveSysType(){
        String name = null;
        try{
            String key = JsonReader.getRealKeyIgnoreCase(jsonRoot, "driveSystem");
            name = jsonRoot.getString(key);
        }catch (JSONException e){
            e.printStackTrace();
        }
        return name;
    }

    public String getNavigationOption() {
        // ToDo
        String navigationOption = null;
        return (navigationOption);
    }

    public String[] getAttachments() {
        // ToDo
        int len = 0;
        String[] attachmentsArr = null;
        try {
            if (jsonRoot.has("attachments")) {
                JSONArray attachs = jsonRoot.getJSONArray("attachments");
                len = attachs.length();
                attachmentsArr = new String[len];
                for (int i = 0; i < len; i++) {
                    attachmentsArr[i] = attachs.getString(i);
                }
            }
            else {
                attachmentsArr = new String[len];
            }
        } catch (JSONException e) {
            e.printStackTrace();
        }

        return (attachmentsArr);
    }
}
