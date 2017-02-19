package org.firstinspires.ftc.teamcode.attachments;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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
    private CRServo buttonServoCR =null;
    private Servo buttonServoLinear=null;
    private ModernRoboticsI2cColorSensor colorSensor1=null;
    public enum BeaconColor {RED, BLUE, NONE}
    public BeaconColor beaconColor;

    private double curLength;
    double buttonServoSpeed; // units: cm per second

    double strokeLength; // units: cm
    public enum BeaconClaimOperation {EXTEND, RETRACT, NONE}
    private BeaconClaimOperation lastOp;
    private ElapsedTime lastOpTimer;

    public BeaconClaim(FTCRobot robot, LinearOpMode curOpMode, JSONObject rootObj) {
        this.curOpMode = curOpMode;
        this.robot = robot;
        String key=null;
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
            key = JsonReader.getRealKeyIgnoreCase(motorsObj, "buttonServoCR");
            if (key == null)
                key = JsonReader.getRealKeyIgnoreCase(motorsObj, "buttonServoLinear");
            buttonServoObj = motorsObj.getJSONObject(key);
            key = JsonReader.getRealKeyIgnoreCase(sensorsObj, "colorSensor1");
            coloSensor1Obj = sensorsObj.getJSONObject(key);

        } catch (JSONException e) {
            e.printStackTrace();
        } catch (NullPointerException e) {
            e.printStackTrace();
        }
        if (buttonServoObj != null) {
            String servoType=null;
            try {
                key = JsonReader.getRealKeyIgnoreCase(buttonServoObj, "motorType");
                servoType = buttonServoObj.getString(key);
            } catch (JSONException e) {
                e.printStackTrace();
            }

            try {
                if (servoType.equalsIgnoreCase("CRservo")) {
                    buttonServoCR = curOpMode.hardwareMap.crservo.get("buttonServo");
                    if (buttonServoObj.getBoolean("needReverse")) {
                        DbgLog.msg("ftc9773: Reversing the button servo");
                        buttonServoCR.setDirection(CRServo.Direction.REVERSE);
                    }
                } else if (servoType.equalsIgnoreCase("LinearServo")) {
                    buttonServoLinear = curOpMode.hardwareMap.servo.get("buttonServo");
                    if (buttonServoObj.getBoolean("needReverse")) {
                        DbgLog.msg("ftc9773: Reversing the button servo");
                        buttonServoLinear.setDirection(Servo.Direction.REVERSE);
                    }
                    key = JsonReader.getRealKeyIgnoreCase(buttonServoObj, "scaleRangeMin");
                    double scaleMin = buttonServoObj.getDouble(key);
                    key = JsonReader.getRealKeyIgnoreCase(buttonServoObj, "scaleRangeMax");
                    double scaleMax = buttonServoObj.getDouble(key);
                    buttonServoLinear.scaleRange(scaleMin, scaleMax);
                    buttonServoLinear.setPosition(0.0);
                }
                // speed and strokeLength parameters are common to both types of servos
                key = JsonReader.getRealKeyIgnoreCase(buttonServoObj, "speed");
                buttonServoSpeed = buttonServoObj.getDouble(key);
                key = JsonReader.getRealKeyIgnoreCase(buttonServoObj, "strokeLength");
                strokeLength = buttonServoObj.getDouble(key);
            } catch (IllegalArgumentException e) {
                DbgLog.msg("ftc9773:  IllegalArgumentException has occurred");
                e.printStackTrace();
            }catch (JSONException e) {
                // Based on tests, it takes 200 milliseconds per 1 cm of extension
                DbgLog.msg("ftc9773: JSON exception occurred.  key =%s", key);
                buttonServoSpeed = 5.0; // default value = 5 cm per second
                strokeLength = 15.0; // Just in case the JSON reader failed, set the default in 15 cm
                e.printStackTrace();
            }

            lastOp = BeaconClaimOperation.NONE;
            lastOpTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
            lastOpTimer.reset();
            DbgLog.msg("ftc9773: buttonServoSpeed=%f, strokeLength=%f",
                    buttonServoSpeed, strokeLength);
        }
        if (coloSensor1Obj != null) {
            colorSensor1 = curOpMode.hardwareMap.get(ModernRoboticsI2cColorSensor.class, "colorSensor1");
            colorSensor1.enableLed(false);
        }

        beaconColor = BeaconColor.NONE;
        curLength = 0.0;
    }

    public double getCurLength() {
        return curLength;
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
    private void updateBeaconServoLength(BeaconClaimOperation op) {
        if (lastOp == BeaconClaimOperation.NONE) {
            lastOpTimer.reset();
        } else if (lastOp == BeaconClaimOperation.EXTEND) {
            curLength += lastOpTimer.milliseconds() * buttonServoSpeed / 1000;
            curLength =  (curLength > strokeLength) ? strokeLength : curLength;
        } else {
            curLength -= lastOpTimer.milliseconds() * buttonServoSpeed / 1000;
            curLength = (curLength < 0) ? 0 : curLength;
        }
        lastOpTimer.reset();
        lastOp = op;
    }
    public void pushBeacon(){
        if (buttonServoCR != null) {
            updateBeaconServoLength(BeaconClaimOperation.EXTEND);
            buttonServoCR.setPower(-1.0);
        } else if (buttonServoLinear != null) {
            lastOp = BeaconClaimOperation.EXTEND;
            curLength = Range.clip(curLength+1.0, 0, strokeLength);
            buttonServoLinear.setPosition(curLength / strokeLength);
        }
    }
    public void retractBeacon(){
        if (buttonServoCR != null) {
            updateBeaconServoLength(BeaconClaimOperation.RETRACT);
            buttonServoCR.setPower(1.0);
        } else if (buttonServoLinear != null) {
            lastOp = BeaconClaimOperation.RETRACT;
            curLength = Range.clip(curLength-1.0, 0, strokeLength);
            buttonServoLinear.setPosition(curLength / strokeLength);
        }
    }
    public void idleBeacon(){
        if (buttonServoCR != null) {
            updateBeaconServoLength(BeaconClaimOperation.NONE);
            buttonServoCR.setPower(0.0);
        } else if (buttonServoLinear != null) {
            lastOp = BeaconClaimOperation.NONE;
        }
    }

    public void activateButtonServo(double timeToExtend, double lengthToExtend) {
        if (buttonServoCR != null) {
            ElapsedTime elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
            elapsedTime.reset();
            while (elapsedTime.milliseconds() < timeToExtend && curOpMode.opModeIsActive()) {
                pushBeacon();
            }
//        curOpMode.sleep(500);
            idleBeacon();
            curLength += (timeToExtend * buttonServoSpeed);
            curLength = (curLength > strokeLength) ? strokeLength : curLength;
        } else if (buttonServoLinear != null) {
            double servoPosition = Range.clip(lengthToExtend, 0, strokeLength) / strokeLength;
            servoPosition = Range.clip(buttonServoLinear.getPosition() + servoPosition, 0, 1);
            DbgLog.msg("ftc9773: activateButtonServo: cur position = %f, new position=%f",
                    buttonServoLinear.getPosition(), servoPosition);
            buttonServoLinear.setPosition(servoPosition);
            curOpMode.sleep((long)timeToExtend);
        }
    }

    public void deactivateButtonServo(double timeToRetract, double lengthToRetract) {
        if (buttonServoCR != null) {
            ElapsedTime elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
            elapsedTime.reset();
            while ((elapsedTime.milliseconds() < timeToRetract) && curOpMode.opModeIsActive()) {
                retractBeacon();
            }
//        curOpMode.sleep(500);
            idleBeacon();
            curLength -= (timeToRetract * buttonServoSpeed);
            curLength = (curLength < 0) ? 0 : curLength;
        } else if (buttonServoLinear != null) {
            double servoPosition = Range.clip(lengthToRetract, 0, strokeLength) / strokeLength;
            servoPosition = Range.clip(buttonServoLinear.getPosition() - servoPosition, 0, 1);
            DbgLog.msg("ftc9773: deactivateButtonServo: cur position = %f, new position=%f",
                    buttonServoLinear.getPosition(), servoPosition);
            buttonServoLinear.setPosition(servoPosition);
            curOpMode.sleep(600);
        }
    }

    public void claimABeacon(double distanceFromWall) {
        double lengthToExtend = distanceFromWall - curLength;
        lengthToExtend =  (lengthToExtend < 0) ? 2 : lengthToExtend;

        double timeToExtend = lengthToExtend * (1000 / buttonServoSpeed);
        DbgLog.msg("ftc9773: timeToExtend=%f millis, lengthToExtend=%f cm",
                timeToExtend, lengthToExtend);
        activateButtonServo(timeToExtend, lengthToExtend);
        curOpMode.sleep(50);
        deactivateButtonServo(timeToExtend, lengthToExtend);
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
        activateButtonServo(1000, 5);
        deactivateButtonServo(1000, 5);
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

    public double getStrokeLength() {
        return strokeLength;
    }

}
