package org.firstinspires.ftc.teamcode.attachments;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
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

public class ParticleRelease implements Attachment{

    FTCRobot robot;
    LinearOpMode curOpMode;
    Servo particleServo;
    
    public ParticleRelease(FTCRobot robot, LinearOpMode curOpMode, JSONObject rootObj){
        String key;
        JSONObject particleObj = null;
        JSONObject motorsObj = null, particleServoObj=null;

        this.robot = robot;
        this.curOpMode = curOpMode;
        try {
            key = JsonReader.getRealKeyIgnoreCase(rootObj, "ParticleRelease");
            particleObj = rootObj.getJSONObject(key);
            key = JsonReader.getRealKeyIgnoreCase(particleObj, "motors");
            motorsObj = particleObj.getJSONObject(key);
            key = JsonReader.getRealKeyIgnoreCase(motorsObj, "particleReleaseServo");
            particleServoObj = motorsObj.getJSONObject(key);
            particleServo = curOpMode.hardwareMap.servo.get("particleReleaseServo");
            particleServo.scaleRange(particleServoObj.getDouble("scaleRangeMin"),
                    particleServoObj.getDouble("scaleRangeMax"));
            if (particleServoObj.getBoolean("needReverse")) {
                DbgLog.msg("ftc9773: Reversing the particle release servo");
                particleServo.setDirection(Servo.Direction.REVERSE);
            }
            particleServo.setPosition(1.0);
        } catch (JSONException e) {
            e.printStackTrace();
        }
    }

    public void releaseParticles() {
        particleServo.setPosition(0.0);
    }

    public void keepParticles() {
        particleServo.setPosition(1.0);
    }
    
    @Override
    public void getAndApplyDScmd() {
        switch (robot.driverStation.getParticleReleaseCmd().status){
            case RELEASE_WAIT_KEEP: takeParticles(); break;
            case KEEP: keepParticles(); break;
        }
    }

    public void takeParticles(){
        releaseParticles();
        curOpMode.sleep(500);
        keepParticles();
    }
}
