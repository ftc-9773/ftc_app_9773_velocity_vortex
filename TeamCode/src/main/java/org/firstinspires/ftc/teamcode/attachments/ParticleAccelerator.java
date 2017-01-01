package org.firstinspires.ftc.teamcode.attachments;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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
    DcMotor launcherMotor;
    long rampUpTime = 2000; // default value in milli seconds

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
                DbgLog.msg("ftc9773: Reversing the launcher motor");
                launcherMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            }
            key = JsonReader.getRealKeyIgnoreCase(launcherMotorObj, "rampUpTime");
            rampUpTime = launcherMotorObj.getLong(key);
            double maxSpeed = launcherMotorObj.getDouble("maxSpeed");
            launcherMotor.setMaxSpeed((int)(launcherMotor.getMaxSpeed() * maxSpeed));
            // Set the zero power behaviour to float sot hat the motor stops gradually
            // This is recommended for high speed low torque motors
            launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
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
        launcherMotor.setPower(1.0);
    }

    public void deactivateParticleAccelerator() {
        // Zero power behaviour was set to FLOAT in the constructor.
        launcherMotor.setPower(0.0);
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
