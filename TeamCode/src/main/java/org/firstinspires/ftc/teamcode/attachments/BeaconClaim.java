package org.firstinspires.ftc.teamcode.attachments;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.FTCRobot;
import org.firstinspires.ftc.teamcode.util.JsonReaders.JsonReader;
import org.firstinspires.ftc.teamcode.navigation.Navigation;
import org.json.JSONException;
import org.json.JSONObject;


/*
 * Copyright (c) 2016 Robocracy 9773
 */

public class BeaconClaim implements Attachment {
    private FTCRobot robot;
    private LinearOpMode curOpMode;
    private CRServo buttonServo=null;
    private Servo colorServo=null;
    private ModernRoboticsI2cColorSensor colorSensor1=null;
    public enum BeaconColor {RED, BLUE, NONE}
    public BeaconColor beaconColor;

    public BeaconClaim(FTCRobot robot, LinearOpMode curOpMode, JSONObject rootObj) {
        this.curOpMode = curOpMode;
        this.robot = robot;
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
            key = JsonReader.getRealKeyIgnoreCase(sensorsObj, "colorSensor1");
            coloSensor1Obj = sensorsObj.getJSONObject(key);

        } catch (JSONException e) {
            e.printStackTrace();
        } catch (NullPointerException e) {
            e.printStackTrace();
        }
        if (buttonServoObj != null) {
            try {
                buttonServo = curOpMode.hardwareMap.crservo.get("buttonServo");
            } catch (IllegalArgumentException e) {
                e.printStackTrace();
            }
        }
        if (coloSensor1Obj != null) {
            colorSensor1 = curOpMode.hardwareMap.get(ModernRoboticsI2cColorSensor.class, "colorSensor1");
            colorSensor1.enableLed(false);
        }
        if (colorServoObj != null) {
            try {
                colorServo = curOpMode.hardwareMap.servo.get("colorServo");
            } catch (IllegalArgumentException e) {
                e.printStackTrace();
            }
        }


        // Set the MIN and MAX positions for servos
        try {
            if (buttonServo != null) {
                if (buttonServoObj.getBoolean("needReverse")) {
                    DbgLog.msg("ftc9773: Reversing the button servo");
                    buttonServo.setDirection(CRServo.Direction.REVERSE);
                }
           }

            if (colorServo != null) {
                colorServo.scaleRange(colorServoObj.getDouble("scaleRangeMin"),
                        colorServoObj.getDouble("scaleRangeMax"));
                if (colorServoObj.getBoolean("needReverse")) {
                    DbgLog.msg("ftc9773: Reversing the color servo");
                    colorServo.setDirection(Servo.Direction.REVERSE);
                }
                colorServo.setPosition(1.0);
            }
        } catch (JSONException e) {
            e.printStackTrace();
        } catch (NullPointerException e) {
            e.printStackTrace();
        }
        beaconColor = BeaconColor.NONE;
    }

    // method to activate or reset beacon claim attachment
    // This method should be called in the while(opModeIsActive) loop
    @Override
    public void getAndApplyDScmd() {
        if (curOpMode.gamepad2.x){
            pushBeacon();
        }
        else if (curOpMode.gamepad2.b){
            retractBeacon();
        }
        else {
            idleBeacon();
        }
    }
    public void pushBeacon(){
        buttonServo.setPower(1.0);
    }
    public void retractBeacon(){
        buttonServo.setPower(-1.0);
    }
    public void idleBeacon(){
        buttonServo.setPower(0.0);
    }

    public void activateButtonServo() {
        ElapsedTime elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        elapsedTime.reset();
        while (elapsedTime.milliseconds() < 1200 && curOpMode.opModeIsActive()) {
            pushBeacon();
        }
//        curOpMode.sleep(500);
        idleBeacon();
    }

    public void deactivateButtonServo() {
        ElapsedTime elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        elapsedTime.reset();
        while (elapsedTime.milliseconds() < 1200 && curOpMode.opModeIsActive()) {
            retractBeacon();
        }
//        curOpMode.sleep(500);
        idleBeacon();
    }

    public void claimABeacon() {
        activateButtonServo();
        curOpMode.sleep(50);
        deactivateButtonServo();
    }

    public void verifyBeaconColor(){
        colorSensor1.enableLed(false);
        curOpMode.telemetry.addData("red: ", "%s", Integer.toString(colorSensor1.red()));
        curOpMode.telemetry.addData("blue: ", "%s", Integer.toString(colorSensor1.blue()));
        curOpMode.telemetry.update();
        DbgLog.msg("ftc9773: red value = %d, blue value = %d",colorSensor1.red(),colorSensor1.blue());
        //DbgLog.msg("color number = %x", colorSensor1.getI2cAddress().get7Bit());
    }

    public void verifyBeaconServo() {
        activateButtonServo();
        deactivateButtonServo();
    }

    public boolean isBeaconRed() {
        int redValue = colorSensor1.red();
        int blueValue = colorSensor1.blue();
        if (redValue >= 3 && ((redValue - blueValue) >= 2)) {
            return true;
        } else {
            return false;
        }
    }

    public boolean isBeaconBlue() {
        int redValue = colorSensor1.red();
        int blueValue = colorSensor1.blue();
        return (blueValue >= 3 && ((blueValue - redValue) >= 2));

    }

    public String checkBeaconColor() {
        DbgLog.msg("ftc9773: red=%d, blue=%d, green=%d", colorSensor1.red(), colorSensor1.blue(),
                colorSensor1.green());
        return null;
    }

    public void setBeaconStatus() {
        beaconColor = (isBeaconBlue() ? BeaconColor.BLUE :
                (isBeaconRed() ? BeaconColor.RED : BeaconColor.NONE));

        DbgLog.msg("ftc9773: Beacon color values: red=%d, blue=%d", colorSensor1.red(), colorSensor1.blue());
        DbgLog.msg("ftc9773: Beacon color detected: %s", (beaconColor == BeaconColor.RED) ? "red" :
                ((beaconColor == BeaconColor.BLUE) ? "blue" : "none"));
    }

    public BeaconColor getBeaconColor() {
        return (beaconColor);
    }

}
