package org.firstinspires.ftc.teamcode.util.JsonReaders;

import org.json.JSONException;
import org.json.JSONObject;

/**
 * Created by ftcrobocracy on 2/13/17.
 */

public class DriverStationReader extends JsonReader {
    public JSONObject drvrStationReader;
    public DriverStationReader(String filePath, String gamepadConfig) {
        super(filePath);
        try {
            this.drvrStationReader = jsonRoot.getJSONObject(gamepadConfig);
        } catch (JSONException e) {
            e.printStackTrace();
        }
    }

    public String getKeyForCmd(String cmd) {
        String gamepadKey=null;
        try {
            String key = JsonReader.getRealKeyIgnoreCase(drvrStationReader, cmd);
            gamepadKey = drvrStationReader.getString(key);
        } catch (JSONException e) {
            e.printStackTrace();
        }
        return (gamepadKey);
    }

    /**
     * Cross check the keysToCmds onject with cmdsToKeys object
     */
    public void crossCheckKeysAndCmds() {

    }
}
