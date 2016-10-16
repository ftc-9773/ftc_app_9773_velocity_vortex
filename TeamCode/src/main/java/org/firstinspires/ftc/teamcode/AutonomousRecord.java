package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drivesys.FourMotorTankDrive;
import org.firstinspires.ftc.teamcode.drivesys.TwoMotorDrive;
import org.firstinspires.ftc.teamcode.drivesys.Wheel;
import org.firstinspires.ftc.teamcode.util.AutonomousRecordReader;
import org.firstinspires.ftc.teamcode.util.DriveSysReader;
import org.firstinspires.ftc.teamcode.util.FileRW;
import org.json.JSONException;
import org.json.JSONObject;


@Autonomous(name = "Record", group = "Autonomous")
public class AutonomousRecord extends LinearOpMode {
    Wheel wheel;
    FileRW fileRW;
    FourMotorTankDrive fourMotorTankDrive;

    @Override
    public void runOpMode() throws InterruptedException {
       // DriveSysReader driveSysReader = new DriveSysReader("/sdcard/FIRST/json/robot_4motor_4wd.json");
        AutonomousRecordReader autonomousRecordReader = new AutonomousRecordReader("/sdcard/FIRST/autonomous_record.json");
        DriveSysReader driveSysReader = autonomousRecordReader.getDriveSysReader();
        String recordFileName = autonomousRecordReader.getRecordFilePath();
        DbgLog.msg("driveSysType=%s", driveSysReader.getDriveSysType());
        try{
            if(driveSysReader.getDriveSysType().equals("4WD")){
                DbgLog.msg("Reached here 1!");

                int numMotors = 4;
                JSONObject motors = driveSysReader.getMotors();
                JSONObject fMotorLObj = motors.getJSONObject("fMotorL");
                JSONObject fMotorRObj = motors.getJSONObject("fMotorR");
                JSONObject rMotorLObj = motors.getJSONObject("rMotorL");
                JSONObject rMotorRObj = motors.getJSONObject("rMotorR");

                DcMotor fMotorL = hardwareMap.dcMotor.get(fMotorLObj.getString("motorName"));
                DcMotor fMotorR = hardwareMap.dcMotor.get(fMotorRObj.getString("motorName"));
                DcMotor rMotorL = hardwareMap.dcMotor.get(rMotorLObj.getString("motorName"));
                DcMotor rMotorR = hardwareMap.dcMotor.get(rMotorRObj.getString("motorName"));
                DbgLog.msg("Reached here 2!");
                if(driveSysReader.getWheelType().equals("rubber-treaded")){
                    Wheel wheel = new Wheel(Wheel.Type.RUBBER_TREADED, driveSysReader.getWheelDiameter());
                }


                fourMotorTankDrive = new FourMotorTankDrive(fMotorL,rMotorL, fMotorR, rMotorR,
                        1, 0, 1.0, wheel, fMotorLObj.getInt("motorCPR"));
                DbgLog.msg("Reached here 3!");
            }
        }catch (JSONException e){
            e.printStackTrace();
        }

        // ToDo:  Get the record-replay file path from the json config file.
        fileRW = new FileRW(recordFileName, true);

        waitForStart();
        //long startingTime = System.nanoTime();
        //long elapsedTime = 0;
        while (opModeIsActive()) {
            //long timestamp = System.nanoTime() - (startingTime + elapsedTime);
            double speed = gamepad1.left_stick_x * 0.5;
            double direction = gamepad1.right_stick_y * 0.5;

            fileRW.fileWrite(Double.toString(speed) + "," + Double.toString(direction));
            fourMotorTankDrive.drive((float) speed, (float) direction);

            DbgLog.msg(String.format("Speed: %f", speed, " , Direction: %f", direction));

            //elapsedTime = System.nanoTime() - startingTime;
            if(gamepad1.a){
                break;
            }
            sleep(5);
        }
        DbgLog.msg("Is close executing?");
        fileRW.close();

    }
}
