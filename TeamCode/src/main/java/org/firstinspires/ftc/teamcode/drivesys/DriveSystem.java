package org.firstinspires.ftc.teamcode.drivesys;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.FTCRobot;
import org.firstinspires.ftc.teamcode.util.JsonReaders.DriveSysReader;
import org.firstinspires.ftc.teamcode.util.JsonReaders.JsonReader;
import org.firstinspires.ftc.teamcode.util.JsonReaders.MotorSpecsReader;
import org.firstinspires.ftc.teamcode.util.JsonReaders.WheelSpecsReader;


public class DriveSystem {
    LinearOpMode curOpMode;
    FTCRobot robot;
    String driveSysType;

    public DriveSystem(LinearOpMode curOpMode, FTCRobot robot, String driveSysType) {
        this.curOpMode = curOpMode;
        this.robot = robot;
        this.driveSysType = driveSysType;
    }

    public DriveSystem() {
    }

    public static DriveSystem createDriveSystem(LinearOpMode curOpMode, FTCRobot robot,
                                                String driveSysType) {
        DriveSystem driveSys = null;
        DriveSysReader driveSysReader = new DriveSysReader(JsonReader.driveSystemsFile, driveSysType);
        DbgLog.msg("driveSysType=%s", driveSysReader.getDriveSysType());
        WheelSpecsReader wheelSpecs = new WheelSpecsReader(JsonReader.wheelSpecsFile,
                driveSysReader.getWheelType());
        if(driveSysType.equals("4Motor4WDSteering")||driveSysType.equals("4Motor6WDSteering")) {
            int CPR = 0;
            double wheelDiameter = 0.0;
            double frictionCoeff = 1.0;
            FourMotorTankDrive fourMotorTankDrive;

            // Assume that all Drive System motors are of same type,
            // and get the CPR just for one motor
            String fMotorL_type = driveSysReader.getMotorType("fMotorL");
            String wheel_type = driveSysReader.getWheelType();
            MotorSpecsReader motorSpecs =
                    new MotorSpecsReader(JsonReader.motorSpecsFile, fMotorL_type);
            CPR = motorSpecs.getCPR();
            wheelDiameter = wheelSpecs.getDiameter();
            frictionCoeff = wheelSpecs.getFrictionCoeff();

            DcMotor fMotorL = curOpMode.hardwareMap.dcMotor.get("fMotorL");
            DcMotor fMotorR = curOpMode.hardwareMap.dcMotor.get("fMotorR");
            DcMotor rMotorL = curOpMode.hardwareMap.dcMotor.get("rMotorL");
            DcMotor rMotorR = curOpMode.hardwareMap.dcMotor.get("rMotorR");

            Wheel wheel = new Wheel(wheel_type, wheelDiameter);

            DbgLog.msg("wheel diameter = %f", wheel.diameter);
            fourMotorTankDrive = new FourMotorTankDrive(fMotorL, rMotorL, fMotorR, rMotorR,
                    1, 0, frictionCoeff, wheel, CPR);
            fourMotorTankDrive.curOpMode = curOpMode;
            fourMotorTankDrive.robot = robot;
            fourMotorTankDrive.driveSysType = driveSysType;
            driveSys = (DriveSystem) fourMotorTankDrive;
        }
        return (driveSys);
    }

    public void drive(float speed, float direction) {return;}
    public void lineFollow(double leftSpeed, double rightSpeed) {return;}
    public void stop() {return;}
}
