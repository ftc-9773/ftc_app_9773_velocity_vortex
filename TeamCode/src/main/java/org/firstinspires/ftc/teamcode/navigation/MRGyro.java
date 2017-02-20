package org.firstinspires.ftc.teamcode.navigation;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.FTCRobot;

/**
 * Created by Kids on 2/12/2017.
 */
public class MRGyro {

    ModernRoboticsI2cGyro gyro;
    LinearOpMode curOpMode;
    FTCRobot robot;

    public MRGyro(FTCRobot robot, LinearOpMode curOpMode){
        this.robot = robot;
        this.curOpMode = curOpMode;
        gyro = (ModernRoboticsI2cGyro)curOpMode.hardwareMap.gyroSensor.get("gyro");
        gyro.calibrate();

        // make sure the gyro is calibrated before continuing
        while (curOpMode.opModeIsActive() && gyro.isCalibrating())  {
            curOpMode.sleep(50);
            curOpMode.idle();
        }
        gyro.resetZAxisIntegrator();
        DbgLog.msg("Done with initializing MR Gyro");
        DbgLog.msg("Current Yaw=%f, Current pitch=%f", gyro.getIntegratedZValue(), gyro.rawX());
    }

    // todo: find out how to return yaw and pitch angles
    // for Navx it is navx_device.getYaw() and navx_device.getPitch()

    public double getYaw() {return gyro.getIntegratedZValue();}

    public double getPitch() {
        return ((double)gyro.rawX());
    }

    public void setRobotOrientation(double targetAngle, double speed, NavigationChecks navigationChecks){
        double curYaw = getYaw();
        double diff = targetAngle - curYaw;
        double angleToTurn = diff > 180 ? diff - 360: diff < -180 ? diff + 360: diff;
        turnRobot(angleToTurn,speed,navigationChecks);
    }

    public void turnRobot(double angle, double speed, NavigationChecks navigationChecks){
        double leftPower = 0.0, rightPower = 0.0;
        double startingYaw, targetYaw, yawDiff;
        double min_angleToTurn = 0.0;
//        LoopStatistics instr = new LoopStatistics();
        if (angle <0 && angle > -360){
            leftPower = speed;
            rightPower = -1*leftPower;
        } else if (angle > 0 && angle < 360){
            rightPower = speed;
            leftPower = -1*rightPower;
        } else if (angle == 0) {
            rightPower = 0;
            leftPower = 0;
        } else{
            DbgLog.msg("ftc9773: angle %f is invalid!", angle);
            return;
        }

        startingYaw = getYaw(); //not sure whether the range returned by getYaw is appropriate
        targetYaw = startingYaw + angle;
        if (targetYaw > 360){
            targetYaw %= 360;
        } else if (targetYaw < 0){
            while (targetYaw < 0){
                targetYaw += 360;
            }
        }
        DbgLog.msg("ftc9773: power left = %f, right = %f",leftPower, rightPower);
        DbgLog.msg("ftc9773: raw Yaw = %f, Starting yaw = %f, targetYaw = %f",
                gyro.getIntegratedZValue(), startingYaw, targetYaw);

//        robot.repActions.startActions();
        while (curOpMode.opModeIsActive() && !navigationChecks.stopNavigation()) {
            this.robot.driveSystem.turnOrSpin(leftPower, rightPower);
//            robot.repActions.repeatActions();
            yawDiff = getYaw() - startingYaw; //warning
            if (yawDiff > min_angleToTurn)
                break;
        }

    }

    public void goStraightPID(boolean driveBackwards, double degrees, float speed){
        double error = 0.0, correction = 0.0;
        double leftSpeed, rightSpeed;
        error = getYaw() - degrees;//again, not sure what range is returned by getYaw()
        if (error > 180){
            error -= 360;
        } else if (error <-180){
            error += 360;
        }
        correction = -error;//warning
        leftSpeed = Range.clip(speed - correction, 0, speed);//warning
        rightSpeed = Range.clip(speed + correction, 0, speed);//warning

        if (!driveBackwards) {
            robot.driveSystem.turnOrSpin(leftSpeed, rightSpeed);
        } else {
            robot.driveSystem.turnOrSpin(-rightSpeed, -leftSpeed);
        }
    }
}