package org.firstinspires.ftc.teamcode.attachments;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.FTCRobot;
import org.firstinspires.ftc.teamcode.util.JsonReaders.JsonReader;
import org.json.JSONException;
import org.json.JSONObject;

/**
 * Created by rsburugula on 10/22/16.
 */

public class Harvester implements Attachment {
    FTCRobot robot;
    LinearOpMode curOpMode;
    DcMotor harvesterMotor;

    public Harvester(FTCRobot robot, LinearOpMode curOpMode, JSONObject rootObj){
        String key;
        JSONObject harvesterObj = null;
        JSONObject motorsObj = null, harvesterMotorObj = null;

        this.robot = robot;
        this.curOpMode = curOpMode;

        try{
            key = JsonReader.getRealKeyIgnoreCase(rootObj, "Harvester");
            harvesterObj = rootObj.getJSONObject(key);
            key = JsonReader.getRealKeyIgnoreCase(harvesterObj, "motors");
            motorsObj = harvesterObj.getJSONObject(key);
            key = JsonReader.getRealKeyIgnoreCase(motorsObj, "harvesterMotor");
            harvesterMotorObj = motorsObj.getJSONObject(key);
            harvesterMotor = curOpMode.hardwareMap.dcMotor.get("harvesterMotor");
            if (harvesterMotorObj.getBoolean("needReverse")) {
                harvesterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            }
        } catch (JSONException e){
            e.printStackTrace();
        }
    }

    @Override
    public void getAndApplyDScmd(){
        float power;

        power = curOpMode.gamepad2.left_stick_y;
        harvesterMotor.setPower(power);
    }
}
