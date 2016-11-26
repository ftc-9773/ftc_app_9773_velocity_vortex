package org.firstinspires.ftc.teamcode.attachments;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
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
    CRServo liftServo;


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
                DbgLog.msg("Reversing the lift servo");
                liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            }
            liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            double maxSpeed = liftMotorObj.getDouble("maxSpeed");
            liftMotor.setMaxSpeed((int)(liftMotor.getMaxSpeed() * maxSpeed));

            key = JsonReader.getRealKeyIgnoreCase(motorsObj, "liftServo");
            liftServoObj = motorsObj.getJSONObject(key);
            liftServo = curOpMode.hardwareMap.crservo.get("liftServo");
//            liftServo.scaleRange(liftServoObj.getDouble("scaleRangeMin"),
//                    liftServoObj.getDouble("scaleRangeMax"));
            liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (JSONException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void getAndApplyDScmd() {
        float power;


        power = -curOpMode.gamepad2.right_stick_y;

        liftMotor.setPower(power);

        if(curOpMode.gamepad2.right_bumper){
            liftMotor.setPower(0.05);
        }
        if(curOpMode.gamepad2.left_bumper){
            liftMotor.setPower(0);
        }

        if (curOpMode.gamepad2.a){
            liftServo.setPower(-1);
        }
        else if (curOpMode.gamepad2.y){
            liftServo.setPower(1);
        }
        else{
            liftServo.setPower(0.0);
        }
    }
}
