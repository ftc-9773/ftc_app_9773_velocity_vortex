package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drivesys.FourMotorTankDrive;
import org.firstinspires.ftc.teamcode.drivesys.Wheel;
import org.firstinspires.ftc.teamcode.util.AutonomousConfigReader;
import org.firstinspires.ftc.teamcode.util.DriveSysReader;
import org.firstinspires.ftc.teamcode.util.FileRW;
import org.json.JSONException;
import org.json.JSONObject;

@Autonomous(name = "Replay", group = "Autonomous")
public class AutonomousReplay extends LinearOpMode {
    DcMotor motorL;
    DcMotor motorR;
    Wheel wheel;
    FileRW fileRW;
    FourMotorTankDrive fourMotorTankDrive;

    @Override
    public void runOpMode() throws InterruptedException{
        AutonomousConfigReader autonomousConfigReader = new AutonomousConfigReader("/sdcard/FIRST/autonomous_config.json");
        DriveSysReader driveSysReader = autonomousConfigReader.getDriveSysReader();
        try{
            if(driveSysReader.getDriveSysType().equals("4WD")){
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

                if(driveSysReader.getWheelType().equals("rubber-treaded")){
                    wheel = new Wheel(Wheel.Type.RUBBER_TREADED, driveSysReader.getWheelDiameter());
                }


                fourMotorTankDrive = new FourMotorTankDrive(fMotorL,rMotorL, fMotorR, rMotorR,
                        1, 0, 1.0, wheel, fMotorLObj.getInt("motorCPR"));
            }
        }catch (JSONException e){
            e.printStackTrace();
        }

        // ToDo:  Get the record-replay file path from the json config file.
        String fileName = null;
        try{
            fileName = autonomousConfigReader.jsonRoot.getString("replayFileName");
        }
        catch(JSONException exception){
            DbgLog.error("Darn it, JSON flipped out again.");
        }
        fileRW = new FileRW(fileName, false);

        waitForStart();
        while (opModeIsActive()){
            String line = fileRW.getNextLine();
            if(line != null) {
                String[] lineElements = line.split(",");
                if(lineElements.length < 2){
                    continue;
                }
                else {
                    //long timestamp = Long.parseLong(lineElements[0]);
                    double speed = Double.parseDouble(lineElements[0]);
                    double direction = Double.parseDouble(lineElements[1]);

                    //sleep(timestamp);
                    fourMotorTankDrive.drive((float) speed, (float) direction);
                }
            }
            else{
                stop();
            }
            sleep(5);
        }
        fileRW.close();
    }
}
