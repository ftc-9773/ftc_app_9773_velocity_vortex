package org.firstinspires.ftc.teamcode.attachments;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.FTCRobot;
import org.firstinspires.ftc.teamcode.util.JsonReaders.JsonReader;
import org.json.JSONException;
import org.json.JSONObject;


public class BeaconClaim implements Attachment {
    FTCRobot robot;
    LinearOpMode curOpMode;
    Servo buttonServo=null;
    Servo colorServo=null;
    ColorSensor colorSensor1=null;

    public BeaconClaim(FTCRobot robot, LinearOpMode curOpMode, JSONObject rootObj) {
        String key;
        JSONObject beaconJsonObj=null;
        JSONObject motorsObj=null, buttonServoObj=null, colorServoObj=null;
        JSONObject sensorsObj = null,coloSensor1Obj=null;

        try {
            key = JsonReader.getRealKeyIgnoreCase(rootObj, "BeaconClaim");
            beaconJsonObj = rootObj.getJSONObject(key);
            key = JsonReader.getRealKeyIgnoreCase(beaconJsonObj, "motors");
            motorsObj = beaconJsonObj.getJSONObject(key);
            key = JsonReader.getRealKeyIgnoreCase(beaconJsonObj, "sensors");
            sensorsObj = beaconJsonObj.getJSONObject(key);
        } catch (JSONException e) {
            e.printStackTrace();
        }

        try {
            key = JsonReader.getRealKeyIgnoreCase(motorsObj, "buttonServo");
            buttonServoObj = motorsObj.getJSONObject(key);
            key = JsonReader.getRealKeyIgnoreCase(motorsObj, "colorDetectServo");
            colorServoObj = motorsObj.getJSONObject(key);
            key = JsonReader.getRealKeyIgnoreCase(sensorsObj, "colorSensor1");
            coloSensor1Obj = sensorsObj.getJSONObject(key);
        } catch (JSONException e) {
            e.printStackTrace();
        }
        buttonServo = curOpMode.hardwareMap.servo.get("buttonServo");
        colorServo = curOpMode.hardwareMap.servo.get("colorDetectServo");
        colorSensor1 = curOpMode.hardwareMap.colorSensor.get("colorSensor1");

        // Set the MIN and MAX positions for servos
        try {
            buttonServo.scaleRange(buttonServoObj.getDouble("scaleRangeMin"),
                    buttonServoObj.getDouble("scaleRangeMax"));
            if (buttonServoObj.getBoolean("needReverse")) {
                buttonServo.setDirection(Servo.Direction.REVERSE);
            }

            colorServo.scaleRange(colorServoObj.getDouble("scaleRangeMin"),
                    colorServoObj.getDouble("scaleRangeMax"));
            if (colorServoObj.getBoolean("needReverse")) {
                colorServo.setDirection(Servo.Direction.REVERSE);
            }

            // Set the initial positions for both the servos
            buttonServo.setPosition(1.0);
            colorServo.setPosition(1.0);
        } catch (JSONException e) {
            e.printStackTrace();
        }
    }

    // method to activate or reset beacon claim attachment
    // This method should be called in the while(opModeIsActive) loop
    @Override
    public void getAndApplyDScmd() {
        if (curOpMode.gamepad1.a) {
            buttonServo.setPosition(0.0);
        }
        if (curOpMode.gamepad1.y) {
            buttonServo.setPosition(1.0);
        }
        if (curOpMode.gamepad1.x) {
            colorServo.setPosition(0.0);
        }
        if (curOpMode.gamepad1.b) {
            colorServo.setPosition(1.0);
        }
    }
}
