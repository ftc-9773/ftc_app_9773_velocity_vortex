package org.firstinspires.ftc.teamcode.attachments;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.FTCRobot;
import org.firstinspires.ftc.teamcode.util.JsonReaders.JsonReader;
import org.json.JSONException;
import org.json.JSONObject;

/**
 * Created by michaelzhou on 11/13/16.
 */

/*
 * Copyright (c) 2016 Robocracy 9773
 */

public class ParticleAccelerator implements Attachment{
    FTCRobot robot;
    LinearOpMode curOpMode;
    public DcMotor launcherMotor1, launcherMotor2;
    long rampUpTime = 2000; // default value in milli seconds

    public ParticleAccelerator(FTCRobot robot, LinearOpMode curOpMode, JSONObject rootObj) {
        String key;
        JSONObject launcherObj = null;
        JSONObject motorsObj = null, launcherMotorObj1 = null, launcherMotorObj2 = null;

        this.robot = robot;
        this.curOpMode = curOpMode;
        try {
            key = JsonReader.getRealKeyIgnoreCase(rootObj, "ParticleAccelerator");
            launcherObj = rootObj.getJSONObject(key);
            key = JsonReader.getRealKeyIgnoreCase(launcherObj, "motors");
            motorsObj = launcherObj.getJSONObject(key);
        } catch (JSONException e) {
            e.printStackTrace();
        }
        try{
            key = JsonReader.getRealKeyIgnoreCase(motorsObj, "partAccMotor1");
            launcherMotorObj1 = motorsObj.getJSONObject(key);
            launcherMotor1 = curOpMode.hardwareMap.dcMotor.get("partAccMotor1");
            if (launcherMotorObj1.getBoolean("needReverse")) {
                DbgLog.msg("ftc9773: Reversing the launcher motor");
                launcherMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
            }
            key = JsonReader.getRealKeyIgnoreCase(launcherMotorObj1, "rampUpTime");
            rampUpTime = launcherMotorObj1.getLong(key);
            launcherMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            int maxSpeed = launcherMotorObj.getInt("maxSpeed");
//            launcherMotor.setMaxSpeed(maxSpeed);
            // Set the zero power behaviour to float sot hat the motor stops gradually
            // This is recommended for high speed low torque motors
            launcherMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        } catch (JSONException e) {
            e.printStackTrace();
        }
        try{
            key = JsonReader.getRealKeyIgnoreCase(motorsObj, "partAccMotor2");
            launcherMotorObj2 = motorsObj.getJSONObject(key);
            launcherMotor2 = curOpMode.hardwareMap.dcMotor.get("partAccMotor2");
            if (launcherMotorObj1.getBoolean("needReverse")) {
                DbgLog.msg("ftc9773: Reversing the launcher motor");
                launcherMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
            }
            key = JsonReader.getRealKeyIgnoreCase(launcherMotorObj2, "rampUpTime");
            rampUpTime = launcherMotorObj2.getLong(key);
            launcherMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            int maxSpeed = launcherMotorObj.getInt("maxSpeed");
//            launcherMotor.setMaxSpeed(maxSpeed);
            // Set the zero power behaviour to float sot hat the motor stops gradually
            // This is recommended for high speed low torque motors
            launcherMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        } catch (JSONException e) {
            e.printStackTrace();
        }
    }

    public void activateParticleAccelerator() {
        // Ramp up the power gradually.
        // This is recommended for high speed low torque motors
//        ElapsedTime rampUpTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
//        rampUpTimer.reset();
//        while ((rampUpTimer.milliseconds() < rampUpTime) && curOpMode.opModeIsActive()) {
//            launcherMotor.setPower(1 - (rampUpTimer.milliseconds() / rampUpTime));
//        }
        launcherMotor1.setPower(1.0);
        launcherMotor2.setPower(1.0);
    }

    public void deactivateParticleAccelerator() {
        // Zero power behaviour was set to FLOAT in the constructor.
        launcherMotor1.setPower(0.0);
        launcherMotor2.setPower(0.0);
    }
    
    @Override
    public void getAndApplyDScmd() {
        if (curOpMode.gamepad1.dpad_up) {
            activateParticleAccelerator();
        }
        else if (curOpMode.gamepad1.dpad_down) {
            deactivateParticleAccelerator();
        }
    }
}
