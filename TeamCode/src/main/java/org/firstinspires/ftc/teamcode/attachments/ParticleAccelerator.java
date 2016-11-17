package org.firstinspires.ftc.teamcode.attachments;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.FTCRobot;
import org.firstinspires.ftc.teamcode.util.JsonReaders.JsonReader;
import org.json.JSONException;
import org.json.JSONObject;

/**
 * Created by michaelzhou on 11/13/16.
 */

public class ParticleAccelerator implements Attachment{
    FTCRobot robot;
    LinearOpMode curOpMode;
    DcMotor launcherMotor;

    public ParticleAccelerator(FTCRobot robot, LinearOpMode curOpMode, JSONObject rootObj) {
        String key;
        JSONObject launcherObj = null;
        JSONObject motorsObj = null, launcherMotorObj = null;

        this.robot = robot;
        this.curOpMode = curOpMode;
        try {
            key = JsonReader.getRealKeyIgnoreCase(rootObj, "ParticleAccelerator");
            launcherObj = rootObj.getJSONObject(key);
            key = JsonReader.getRealKeyIgnoreCase(launcherObj, "motors");
            motorsObj = launcherObj.getJSONObject(key);
            key = JsonReader.getRealKeyIgnoreCase(motorsObj, "partAccMotor");
            launcherMotorObj = motorsObj.getJSONObject(key);
            launcherMotor = curOpMode.hardwareMap.dcMotor.get("partAccMotor");
            if (launcherMotorObj.getBoolean("needReverse")) {
                DbgLog.msg("Reversing the launcher motor");
                launcherMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            }
            double maxSpeed = launcherMotorObj.getDouble("maxSpeed");
            launcherMotor.setMaxSpeed((int)(launcherMotor.getMaxSpeed() * maxSpeed));
        } catch (JSONException e) {
            e.printStackTrace();
        }
    }
    
    @Override
    public void getAndApplyDScmd() {
        float power;
        power = curOpMode.gamepad2.right_stick_y;
        power = Range.clip(power, -1, 1);
        launcherMotor.setPower(power);
    }
}
