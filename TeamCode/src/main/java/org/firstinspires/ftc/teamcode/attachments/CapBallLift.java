package org.firstinspires.ftc.teamcode.attachments;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.FTCRobot;
import org.firstinspires.ftc.teamcode.util.JsonReaders.JsonReader;
import org.json.JSONException;
import org.json.JSONObject;


public class CapBallLift implements  Attachment {
    FTCRobot robot;
    LinearOpMode curOpMode;
    DcMotor liftMotor;
    Servo liftServo;


    public CapBallLift(FTCRobot robot, LinearOpMode curOpMode, JSONObject rootObj) {
        String key;
        JSONObject liftObj = null;
        JSONObject motorsObj = null, liftMotorObj = null, liftServoObj=null;

        this.robot = robot;
        this.curOpMode = curOpMode;
        try {
            key = JsonReader.getRealKeyIgnoreCase(rootObj, "CapBallLift");
            liftObj = rootObj.getJSONObject(key);
            key = JsonReader.getRealKeyIgnoreCase(liftObj, "motors");
            motorsObj = liftObj.getJSONObject(key);
            key = JsonReader.getRealKeyIgnoreCase(motorsObj, "liftMotor");
            liftMotorObj = motorsObj.getJSONObject(key);
            liftMotor = curOpMode.hardwareMap.dcMotor.get("liftMotor");
            if (liftMotorObj.getBoolean("needReverse")) {
                liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            }
            key = JsonReader.getRealKeyIgnoreCase(motorsObj, "liftServo");
            liftServoObj = motorsObj.getJSONObject(key);
            liftServo = curOpMode.hardwareMap.servo.get("liftServo");
            liftServo.scaleRange(liftServoObj.getDouble("scaleRangeMin"),
                    liftServoObj.getDouble("scaleRangeMax"));

        } catch (JSONException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void getAndApplyDScmd() {
        float power;

        power = curOpMode.gamepad2.right_stick_y;
        liftMotor.setPower(power);
        if (curOpMode.gamepad2.y){
            liftServo.setPosition(0.0);
        }
        else if (curOpMode.gamepad2.a){
            liftServo.setPosition(1.0);
        }
    }
}
