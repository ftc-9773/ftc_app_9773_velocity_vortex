package org.firstinspires.ftc.teamcode.attachments;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.InstantRunDexHelper;

import org.firstinspires.ftc.teamcode.FTCRobot;
import org.firstinspires.ftc.teamcode.attachments.Attachment;
import org.firstinspires.ftc.teamcode.util.JsonReaders.JsonReader;
import org.json.JSONException;
import org.json.JSONObject;


public class BeaconClaim implements Attachment {
    private FTCRobot robot;
    private LinearOpMode curOpMode;
    private CRServo buttonServo=null;
    private Servo colorServo=null;
    private ColorSensor colorSensor1=null;
    private TouchSensor touchSensor1=null;
    private I2cAddr i2cAddr=null;
    public boolean[] beaconClaimed;
    public String[] beaconColor;
    public int[] numBlueDetected;
    public int[] numRedDetected;
    public int[] numPressesNeeded;

    public BeaconClaim(FTCRobot robot, LinearOpMode curOpMode, JSONObject rootObj) {
        this.curOpMode = curOpMode;
        this.robot = robot;
        this.i2cAddr = new I2cAddr(0x04);
        String key;
        JSONObject beaconJsonObj=null;
        JSONObject motorsObj=null, buttonServoObj=null, colorServoObj=null;
        JSONObject sensorsObj = null,coloSensor1Obj=null, touchSensor1Obj=null;

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
//            key = JsonReader.getRealKeyIgnoreCase(motorsObj, "colorServo");
//            colorServoObj = motorsObj.getJSONObject(key);
            key = JsonReader.getRealKeyIgnoreCase(sensorsObj, "colorSensor1");
            coloSensor1Obj = sensorsObj.getJSONObject(key);
//            key = JsonReader.getRealKeyIgnoreCase(sensorsObj, "touchSensor1");
//            touchSensor1Obj = sensorsObj.getJSONObject(key);

        } catch (JSONException e) {
            e.printStackTrace();
        } catch (NullPointerException e) {
            e.printStackTrace();
        }
        if (buttonServoObj != null) {
            try {
//                buttonServo = curOpMode.hardwareMap.servo.get("buttonServo");
                buttonServo = curOpMode.hardwareMap.crservo.get("buttonServo");
            } catch (IllegalArgumentException e) {
                e.printStackTrace();
            }
        }
        if (coloSensor1Obj != null) {
            colorSensor1 = curOpMode.hardwareMap.colorSensor.get("colorSensor1");
            colorSensor1.enableLed(false);
        }
        if (touchSensor1Obj != null) {
            touchSensor1 = curOpMode.hardwareMap.touchSensor.get("touchSensor1");
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
                // CR Servo should be set to 0 to stop moving
//                deactivateButtonServo();
//                buttonServo.scaleRange(buttonServoObj.getDouble("scaleRangeMin"),
//                        buttonServoObj.getDouble("scaleRangeMax"));
//                if (buttonServoObj.getBoolean("needReverse")) {
//                    DbgLog.msg("Reversing the button servo");
//                    buttonServo.setDirection(Servo.Direction.REVERSE);
//                }
//
//                // Set the initial positions for both the servos
//                buttonServo.setPosition(1.0);
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
    }

    // method to activate or reset beacon claim attachment
    // This method should be called in the while(opModeIsActive) loop
    @Override
    public void getAndApplyDScmd() {
        if (curOpMode.gamepad2.x) {
            buttonServo.setPower(-1.0);
        }
        else if (curOpMode.gamepad2.b) {
            buttonServo.setPower(1.0);
        }
        else {
            buttonServo.setPower(0.0);
        }
    }

    public void activateButtonServo() {
        ElapsedTime elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        elapsedTime.reset();
        while (elapsedTime.milliseconds() < 1000) {
            buttonServo.setPower(1.0);
        }
//        curOpMode.sleep(500);
        buttonServo.setPower(0.0);
    }

    public void deactivateButtonServo() {
        ElapsedTime elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        elapsedTime.reset();
        while (elapsedTime.milliseconds() < 1000) {
            buttonServo.setPower(-1.0);
        }
//        curOpMode.sleep(500);
        buttonServo.setPower(0.0);
    }

    public void claimABeacon(int beaconId) {
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

    public void claimABeaconV2() {
        activateButtonServo();
        deactivateButtonServo();
    }

    public void verifyBeaconColor(){
//       if (robot.autonomousActions.allianceColor.equals("red")) {
//           colorSensor1.red();
//       }
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
        if (colorSensor1.red() > colorSensor1.blue()) {
//            DbgLog.msg("Red");
            return (true);
        }
        else {
//            DbgLog.msg("Not red");
            return (false);
        }
    }

    public boolean isBeaconBlue() {
        if (colorSensor1.blue() > colorSensor1.red()) {
//            DbgLog.msg("Blue");
            return (true);
        }
        else {
//            DbgLog.msg("Not blue");
            return (false);
        }
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

    public void setBeaconStatusV2(int beaconId, String allianceColor) {
        DbgLog.msg("setBeaconStatus: beaconID=%d, allianceColor=%s",
                beaconId, allianceColor);

        if (this.isBeaconBlue()) {
            beaconColor[beaconId-1] = "blue";
        } else if (this.isBeaconRed()) {
            beaconColor[beaconId-1] = "red";
        }

        if (beaconColor[beaconId-1].equalsIgnoreCase(allianceColor)) {
            numPressesNeeded[beaconId-1] = 1;
        } else {
            numPressesNeeded[beaconId-1] = 2;
        }
        DbgLog.msg("setBeaconStatus: numPressesNeeded=%d", numPressesNeeded[beaconId-1]);
    }
}
