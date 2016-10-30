package org.firstinspires.ftc.teamcode.attachments;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.FTCRobot;
import org.firstinspires.ftc.teamcode.util.JsonReaders.JsonReader;
import org.json.JSONException;
import org.json.JSONObject;


public class BeaconClaim implements Attachment {
    FTCRobot robot;
    LinearOpMode curOpMode;
    Servo buttonServo=null;
    ColorSensor colorSensor1=null;
    TouchSensor touchSensor1=null;
    ModernRoboticsI2cRangeSensor rangeSensor1 = null;

    public BeaconClaim(FTCRobot robot, LinearOpMode curOpMode, JSONObject rootObj) {
        this.curOpMode = curOpMode;
        this.robot = robot;
        String key;
        JSONObject beaconJsonObj=null;
        JSONObject motorsObj=null, buttonServoObj=null, colorServoObj=null;
        JSONObject sensorsObj = null,coloSensor1Obj=null, touchSensor1Obj=null, rangeSensor1Obj=null;

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
            key = JsonReader.getRealKeyIgnoreCase(sensorsObj, "touchSensor1");
            touchSensor1Obj = sensorsObj.getJSONObject(key);
            key = JsonReader.getRealKeyIgnoreCase(sensorsObj, "rangeSensor1");
            rangeSensor1Obj = sensorsObj.getJSONObject(key);
        } catch (JSONException e) {
            e.printStackTrace();
        }
        buttonServo = curOpMode.hardwareMap.servo.get("buttonServo");
        colorSensor1 = curOpMode.hardwareMap.colorSensor.get("colorSensor1");
        colorSensor1.enableLed(false);
        touchSensor1 = curOpMode.hardwareMap.touchSensor.get("touchSensor1");
        rangeSensor1 = curOpMode.hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeSensor1");

        //colorSensor1.setI2cAddress(i2cAddr);

        // Set the MIN and MAX positions for servos
        try {
            buttonServo.scaleRange(buttonServoObj.getDouble("scaleRangeMin"),
                    buttonServoObj.getDouble("scaleRangeMax"));
            if (buttonServoObj.getBoolean("needReverse")) {
                DbgLog.msg("Reversing the button servo");
                buttonServo.setDirection(Servo.Direction.REVERSE);
            }

            // Set the initial positions for both the servos
            buttonServo.setPosition(1.0);
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
    }

    public void claimABeacon(){
        // Move back for 1 sec, set the button servo, Check the color
        if (robot.beaconClaimObj.isBeaconBlue()){
            curOpMode.telemetry.addData("%s", "blue sensed");
            curOpMode.telemetry.update();
        } else if (robot.beaconClaimObj.isBeaconRed()){
            curOpMode.telemetry.addData("%s", "red sensed");
            curOpMode.telemetry.update();
        }
        else {
            curOpMode.telemetry.addData("%s", "no color sensed");
            curOpMode.telemetry.update();
        }
        if((robot.autonomousActions.allianceColor.equalsIgnoreCase("blue") && robot.beaconClaimObj.isBeaconBlue()) ||
                (robot.autonomousActions.allianceColor.equalsIgnoreCase("red") && robot.beaconClaimObj.isBeaconRed())){

            for (int i=0;i<2;i++){
                robot.driveSystem.drive((float) -0.5,0);
                curOpMode.sleep(500);
                robot.driveSystem.stop();
                buttonServo.setPosition(0.0);
                robot.driveSystem.drive((float) 0.5,0);
                curOpMode.sleep(500);
                robot.driveSystem.stop();
                curOpMode.sleep(5000);
            }

        } else if((robot.autonomousActions.allianceColor.equalsIgnoreCase("red") && robot.beaconClaimObj.isBeaconBlue()) ||
            (robot.autonomousActions.allianceColor.equalsIgnoreCase("blue") && robot.beaconClaimObj.isBeaconRed())){
            robot.driveSystem.drive((float) -0.5,0);
            curOpMode.sleep(500);
            robot.driveSystem.stop();
            buttonServo.setPosition(0.0);
            robot.driveSystem.drive((float) 0.5,0);
            curOpMode.sleep(500);
            robot.driveSystem.stop();
        }
    }
    public void resetButtonServo(){
        buttonServo.setPosition(1.0);
    }
    public void verifyBeaconColor(){
//       if (robot.autonomousActions.allianceColor.equals("red")){
//           colorSensor1.red();
//       }
        colorSensor1.enableLed(false);
        DbgLog.msg("red value = %d, blue value = %d",colorSensor1.red(),colorSensor1.blue());
        //DbgLog.msg("color number = %x", colorSensor1.getI2cAddress().get7Bit());
    }

    public boolean touchSensorPressed() {
        return (touchSensor1.isPressed());
    }

    public boolean rangeSensorLessThanMinimum(double minimum){
        double curDistance;
        curDistance = rangeSensor1.getDistance(DistanceUnit.CM);
        boolean isLessThanMinimum = false;

        if(curDistance <= minimum){
            isLessThanMinimum = true;
        }
        else {
            isLessThanMinimum = false;
        }

        return isLessThanMinimum;
    }

    public boolean isBeaconRed() {
        if (colorSensor1.red() > colorSensor1.blue()) {
            return (true);
        }
        else {
            return (false);
        }
    }

    public boolean isBeaconBlue() {
        if (colorSensor1.blue() > colorSensor1.red()) {
            return (true);
        }
        else {
            return (false);
        }
    }

}
