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
import org.firstinspires.ftc.teamcode.util.Vision;
import org.json.JSONException;
import org.json.JSONObject;

import org.firstinspires.ftc.robotcontroller.internal.vision.BeaconColorResult;


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
    private Vision vision=null;
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
        this.vision = new Vision(curOpMode, robot);
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
            //key = JsonReader.getRealKeyIgnoreCase(sensorsObj, "colorSensor1");
            //coloSensor1Obj = sensorsObj.getJSONObject(key);

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
        /*if (coloSensor1Obj != null) {
            colorSensor1 = curOpMode.hardwareMap.get(ModernRoboticsI2cColorSensor.class, "colorSensor1");
            colorSensor1.enableLed(false);
            // Create an FTCi2cDeviceState object for the color sensor
            colorSensorState = new FTCi2cDeviceState(colorSensor1);
        }*/
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
        //colorSensorState.setEnabled(true);
    }
    public void disableColorSensor() {
        //colorSensorState.setEnabled(false);
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
        DbgLog.msg("ftc9773: beaconID=%d, numPressesNeeded=%d", beaconId, numPressesNeeded[beaconId-1]);
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
        BeaconColor[] beaconColors = this.getBeaconColor();
        curOpMode.telemetry.addData("left: ", "%s", beaconColors[0].toString());
        curOpMode.telemetry.addData("right: ", "%s", beaconColors[1].toString());
        curOpMode.telemetry.update();
        DbgLog.msg("ftc9773: left value = %s, right value = %s",beaconColors[0].toString(),beaconColors[1].toString());
        //DbgLog.msg("ftc9773: color number = %x", colorSensor1.getI2cAddress().get7Bit());
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
        DbgLog.msg("ftc9773: red=%d, blue=%d, green=%d", colorSensor1.red(), colorSensor1.blue(),
                colorSensor1.green());
        return null;
    }

    public void setBeaconStatus(int beaconId, String allianceColor, int numBlues, int numReds) {
        DbgLog.msg("ftc9773: setBeaconStatus: beaconID=%d, allianceColor=%s, numBlues=%d, numReds=%d",
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
        DbgLog.msg("ftc9773: setBeaconStatus: numPressesNeeded=%d", numPressesNeeded[beaconId-1]);
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

    public BeaconColor[] getBeaconColor() {
        BeaconColor[] beaconColors = new BeaconColor[2];
        BeaconColorResult.BeaconColor[] visionBeaconColors = vision.getVisionBeaconColors();
        if (visionBeaconColors[0] == BeaconColorResult.BeaconColor.RED) {
            beaconColors[0] = BeaconColor.RED;
        } else if (visionBeaconColors[0] == BeaconColorResult.BeaconColor.BLUE) {
            beaconColors[0] = BeaconColor.BLUE;
        } else if (visionBeaconColors[0] == (BeaconColorResult.BeaconColor.GREEN) || (visionBeaconColors[0] == BeaconColorResult.BeaconColor.UNKNOWN)){
            beaconColors[0] = BeaconColor.NONE;
        }
        if (visionBeaconColors[1] == BeaconColorResult.BeaconColor.RED) {
            beaconColors[1] = BeaconColor.RED;
        } else if (visionBeaconColors[1] == BeaconColorResult.BeaconColor.BLUE) {
            beaconColors[1] = BeaconColor.BLUE;
        } else if (visionBeaconColors[1] == (BeaconColorResult.BeaconColor.GREEN) || (visionBeaconColors[1] == BeaconColorResult.BeaconColor.UNKNOWN)){
            beaconColors[1] = BeaconColor.NONE;
        }
        return beaconColors;
    }

    public void printBeaconScanningData() {
        DbgLog.msg("ftc9773: accumulatedRed = %f, accumulatedBlue=%f, firstTimeStamp=%f, lastTimeStamp=%f",
                accumulatedRedValue, accumulatedBlueValue,
                firstDetectedTimeStamp, lastDetectedTimeStamp);
    }

}
