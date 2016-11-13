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
//            key = JsonReader.getRealKeyIgnoreCase(motorsObj, "liftMotor");
//            liftMotorObj = motorsObj.getJSONObject(key);
//            liftMotor = curOpMode.hardwareMap.dcMotor.get("liftMotor");
//            if (liftMotorObj.getBoolean("needReverse")) {
//                DbgLog.msg("Reversing the lift servo");
//                liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//            }
            key = JsonReader.getRealKeyIgnoreCase(motorsObj, "particleServo");
            particleServoObj = motorsObj.getJSONObject(key);
            particleServo = curOpMode.hardwareMap.servo.get("particleServo");
            particleServo.scaleRange(particleServoObj.getDouble("scaleRangeMin"),
                    particleServoObj.getDouble("scaleRangeMax"));

        } catch (JSONException e) {
            e.printStackTrace();
        }
    }
    
    @Override
    public void getAndApplyDScmd() {
        //gamepad TBD TODO:
        if (curOpMode.gamepad1.a) {
            particleServo.setPosition(0.0);
        }
        if (curOpMode.gamepad1.y) {
            particleServo.setPosition(1.0);
        }
    }
}
