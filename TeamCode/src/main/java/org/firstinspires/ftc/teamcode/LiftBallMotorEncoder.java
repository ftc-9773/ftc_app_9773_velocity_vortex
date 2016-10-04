package org.firstinspires.ftc.teamcode;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;

import org.json.*;

/**
 * Created by michaelzhou on 10/4/16.
 */

public class LiftBallMotorEncoder {
    String motorType, motorID, name;
    double minPower, maxPower, maxCPR;

    public void parseJson() throws JSONException {
        String jsonStr = "{\n" +
                "“Motors”: [{\n" +
                "        \"motor type\": \"dc\",\n" +
                "“motorID”: “neverest40”,\n" +
                "        \"name\": \"motorL\",\n" +
                "        \"Min power\": -1,\n" +
                "“Max power”: 1,\n" +
                "“Max cpr”: “1120”\n" +
                "}, \n" +
                "{\n" +
                "\"motor type\": \"dc\",\n" +
                "“motorID”: “neverest40”,\n" +
                "        \"name\": \"motorR\",\n" +
                "        \"Min power\": -1,\n" +
                "“Max power”: 1,\n" +
                "“Max cpr”: “1120”\n" +
                "}]\n" +
                "}";


        JSONObject rootObj = new JSONObject(jsonStr);
        JSONArray motors = rootObj.optJSONArray("Motors");

        String testOutput = "";
        for(int i=0; i<motors.length(); i++){//get the json objects
            JSONObject obj = motors.getJSONObject(i);
            motorType = obj.optString("motor type").toString();
            motorID = obj.optString("motorID").toString();
            name = obj.optString("name").toString();
            minPower = Integer.parseInt(obj.optString("Min power").toString());
            maxPower = Integer.parseInt(obj.optString("Max power").toString());
            maxCPR = Integer.parseInt(obj.optString("Max cpr").toString());
        }

    }

}
