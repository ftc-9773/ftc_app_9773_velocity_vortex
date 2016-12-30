package org.firstinspires.ftc.teamcode.attachments;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.FTCRobot;
import org.firstinspires.ftc.teamcode.util.FTCi2cDeviceState;
import org.firstinspires.ftc.teamcode.util.JsonReaders.JsonReader;
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
    private I2cAddr i2cAddr=null;
    public boolean[] beaconClaimed;
    public String[] beaconColor;
    public int[] numBlueDetected;
    public int[] numRedDetected;
    public int[] numPressesNeeded;
    FTCi2cDeviceState colorSensorState;
    double accumulatedRedValue, accumulatedBlueValue;
    double firstDetectedTimeStamp, lastDetectedTimeStamp;
    ElapsedTime beaconScanTimer;
    public enum BeaconColor {RED, BLUE, NONE}

    public BeaconClaim(FTCRobot robot, LinearOpMode curOpMode, JSONObject rootObj) {
        this.curOpMode = curOpMode;
        this.robot = robot;
        this.i2cAddr = new I2cAddr(0x04);
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
            // Create an FTCi2cDeviceState object for the color sensor
            colorSensorState = new FTCi2cDeviceState(colorSensor1);
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
                    DbgLog.msg("Reversing the button servo");
                    buttonServo.setDirection(CRServo.Direction.REVERSE);
                }
           }

            if (colorServo != null) {
                colorServo.scaleRange(colorServoObj.getDouble("scaleRangeMin"),
                        colorServoObj.getDouble("scaleRangeMax"));
                if (colorServoObj.getBoolean("needReverse")) {
                    DbgLog.msg("Reversing the color servo");
                    colorServo.setDirection(Servo.Direction.REVERSE);
                }
                colorServo.setPosition(1.0);
            }
        } catch (JSONException e) {
            e.printStackTrace();
        } catch (NullPointerException e) {
            e.printStackTrace();
        }

        // Initialize the beacon status variables
        beaconClaimed = new boolean[2];
        beaconColor = new String[2];
        numBlueDetected = new int[2];
        numRedDetected = new int[2];
        numPressesNeeded = new int[2];
        for (int i=0; i<2; i++) {
            beaconClaimed[i] = false;
            beaconColor[i] = "unknown";
            numBlueDetected[i] = 0;
            numRedDetected[i] = 0;
            numPressesNeeded[i] = 0;
        }

        // Initialize attributes for beacon color scanning
        beaconScanTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        accumulatedRedValue = accumulatedBlueValue = 0;
        firstDetectedTimeStamp = Integer.MAX_VALUE;
        lastDetectedTimeStamp = Integer.MIN_VALUE;
    }

    // method to activate or reset beacon claim attachment
    // This method should be called in the while(opModeIsActive) loop
    @Override
    public void getAndApplyDScmd() {
       buttonServo.setPower(curOpMode.gamepad2.x ? -1.0 : curOpMode.gamepad2.b ? 1.0 : 0.0);
    }

    public void activateButtonServo() {
        ElapsedTime elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        elapsedTime.reset();
        while (elapsedTime.milliseconds() < 1000 && curOpMode.opModeIsActive()) {
            buttonServo.setPower(1.0);
        }
//        curOpMode.sleep(500);
        buttonServo.setPower(0.0);
    }

    public void enableColorSensor() {
        colorSensorState.setEnabled(true);
        DbgLog.msg("Enabled color sensor");
    }
    public void disableColorSensor() {
        colorSensorState.setEnabled(false);
        DbgLog.msg("Disabled color sensor");
    }

    public void deactivateButtonServo() {
        ElapsedTime elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        elapsedTime.reset();
        while (elapsedTime.milliseconds() < 1000 && curOpMode.opModeIsActive()) {
            buttonServo.setPower(-1.0);
        }
//        curOpMode.sleep(500);
        buttonServo.setPower(0.0);
    }

    public void claimABeaconOld(int beaconId) {
        // Do different actions based on whether the beacon button has to be pressed once or twice.
        DbgLog.msg("beaconID=%d, numPressesNeeded=%d", beaconId, numPressesNeeded[beaconId-1]);
        if (numPressesNeeded[beaconId-1] == 1) {
            activateButtonServo();
            deactivateButtonServo();
        } else if (numPressesNeeded[beaconId-1] == 2) {
            activateButtonServo();
            deactivateButtonServo();
            curOpMode.sleep(5000);
            activateButtonServo();
            deactivateButtonServo();
        }
        curOpMode.sleep(100);
    }

    public void claimABeacon() {
        activateButtonServo();
        deactivateButtonServo();
    }

    public void verifyBeaconColor(){
        this.enableColorSensor();
        colorSensor1.enableLed(false);
        curOpMode.telemetry.addData("red: ", "%s", Integer.toString(colorSensor1.red()));
        curOpMode.telemetry.addData("blue: ", "%s", Integer.toString(colorSensor1.blue()));
        curOpMode.telemetry.update();
        DbgLog.msg("red value = %d, blue value = %d",colorSensor1.red(),colorSensor1.blue());
        //DbgLog.msg("color number = %x", colorSensor1.getI2cAddress().get7Bit());
    }

    public void verifyBeaconServo() {
        activateButtonServo();
        deactivateButtonServo();
    }

    public boolean isBeaconRed() {
        return colorSensor1.red() > colorSensor1.blue();
    }

    public boolean isBeaconBlue() {
        return colorSensor1.blue() > colorSensor1.red();
    }

    public String checkBeaconColor() {
        DbgLog.msg("red=%d, blue=%d, green=%d", colorSensor1.red(), colorSensor1.blue(),
                colorSensor1.green());
        return null;
    }

    public void setBeaconStatus(int beaconId, String allianceColor, int numBlues, int numReds) {
        DbgLog.msg("setBeaconStatus: beaconID=%d, allianceColor=%s, numBlues=%d, numReds=%d",
                beaconId, allianceColor, numBlues, numReds);
        numBlueDetected[beaconId-1] = numBlues;
        numRedDetected[beaconId-1] = numReds;
        // If numBlues >>>>> numReds
        if ((numBlues - numReds > 100) || (this.isBeaconBlue())) {
            beaconColor[beaconId-1] = "blue";
        } else if ((numReds - numBlues > 100) || this.isBeaconRed()) {
            beaconColor[beaconId-1] = "red";
        }

        if (beaconColor[beaconId-1].equalsIgnoreCase(allianceColor)) {
            numPressesNeeded[beaconId-1] = 1;
        } else {
            numPressesNeeded[beaconId-1] = 2;
        }
        DbgLog.msg("setBeaconStatus: numPressesNeeded=%d", numPressesNeeded[beaconId-1]);
    }

    public void startBeaconScanning() {
        beaconScanTimer.reset();
        accumulatedRedValue = accumulatedBlueValue = 0;
        firstDetectedTimeStamp = Integer.MAX_VALUE;
        lastDetectedTimeStamp = Integer.MIN_VALUE;
        this.enableColorSensor();
    }

    public void stopBeaconScanning() {
        this.disableColorSensor();
    }

    public void updateBeaconScanValues() {
        double millis = beaconScanTimer.milliseconds();
        if (this.isBeaconBlue()) {
            accumulatedBlueValue += millis;
            if (firstDetectedTimeStamp > millis) {
                firstDetectedTimeStamp = millis;
            } else if (lastDetectedTimeStamp < millis) {
                lastDetectedTimeStamp = millis;
            }
        } else if (this.isBeaconRed()) {
            accumulatedRedValue += millis;
            if (firstDetectedTimeStamp > millis) {
                firstDetectedTimeStamp = millis;
            } else if (lastDetectedTimeStamp < millis) {
                lastDetectedTimeStamp = millis;
            }
        }
    }

    public BeaconColor getBeaconColor(String firstOrSecond) {
        switch (firstOrSecond) {
            case "first" : {
                if (accumulatedRedValue < accumulatedBlueValue) {
                    return BeaconColor.RED;
                } else if (accumulatedBlueValue < accumulatedRedValue) {
                    return BeaconColor.BLUE;
                }
                break;
            }
            case "second" : {
                if (accumulatedRedValue > accumulatedBlueValue) {
                    return BeaconColor.RED;
                } else if (accumulatedBlueValue > accumulatedRedValue) {
                    return BeaconColor.BLUE;
                }
                break;
            }
            default: return BeaconColor.NONE;
        }
        return BeaconColor.NONE;
    }

    public void printBeaconScanningData() {
        DbgLog.msg("accumulatedRed = %f, accumulatedBlue=%f, firstTimeStamp=%f, lastTimeStamp=%f",
                accumulatedRedValue, accumulatedBlueValue,
                firstDetectedTimeStamp, lastDetectedTimeStamp);
    }

}
