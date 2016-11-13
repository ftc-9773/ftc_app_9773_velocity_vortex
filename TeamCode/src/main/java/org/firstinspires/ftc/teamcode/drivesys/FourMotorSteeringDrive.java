package org.firstinspires.ftc.teamcode.drivesys;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class FourMotorSteeringDrive extends DriveSystem {
    DcMotor motorL1 = null;
    DcMotor motorL2 = null;
    DcMotor motorR1 = null;
    DcMotor motorR2 = null;
    int motorL1MaxSpeed = 0;
    int motorL2MaxSpeed = 0;
    int motorR1MaxSpeed = 0;
    int motorR2MaxSpeed = 0;
    double frictionCoefficient;
    double maxSpeed;
    double minSpeed;
    Wheel wheel;
    int motorCPR;  // Cycles Per Revolution.  == 1120 for Neverest40, 560 for Neverest20

    public FourMotorSteeringDrive(DcMotor motorL1, DcMotor motorL2, DcMotor motorR1, DcMotor motorR2,
                                  double maxSpeed, double minSpeed, double frictionCoefficient,
                                  Wheel wheel, int motorCPR){
        this.motorL1 = motorL1;
        this.motorL2 = motorL2;
        this.motorR1 = motorR1;
        this.motorR2 = motorR2;
        this.motorR1.setDirection(DcMotorSimple.Direction.REVERSE);
        this.motorR2.setDirection(DcMotorSimple.Direction.REVERSE);
        this.setDriveSysMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motorL1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motorL2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motorR1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motorR2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.frictionCoefficient = frictionCoefficient;
        this.maxSpeed = maxSpeed;
        this.minSpeed = minSpeed;
        this.motorL1MaxSpeed = this.motorL1.getMaxSpeed();
        this.motorL2MaxSpeed = this.motorL2.getMaxSpeed();
        this.motorR1MaxSpeed = this.motorR1.getMaxSpeed();
        this.motorR2MaxSpeed = this.motorR2.getMaxSpeed();
        this.wheel = wheel;
        this.motorCPR = motorCPR;
    }

    @Override
    public void drive(float speed, float direction){
        double left = (speed + direction) * frictionCoefficient;
        double right = (speed - direction) * frictionCoefficient;

        motorL1.setPower(left);
        motorL2.setPower(left);
        motorR1.setPower(right);
        motorR2.setPower(right);
    }

    @Override
    public void turnOrSpin(double leftSpeed, double rightSpeed) {
        motorL1.setPower(leftSpeed);
        motorL2.setPower(leftSpeed);
        motorR1.setPower(rightSpeed);
        motorR2.setPower(rightSpeed);
    }

    @Override
    public void stop() {
        motorL1.setPower(0.0);
        motorL2.setPower(0.0);
        motorR1.setPower(0.0);
        motorR2.setPower(0.0);
    }

    @Override
    public void driveToDistance(float speed, double distanceInInches) {
        this.setMaxSpeed(speed);

        double countsPerInch = motorCPR / wheel.getCircumference();
        double targetCounts = countsPerInch * distanceInInches;

        DbgLog.msg("motorL1 current position = %d", motorL1.getCurrentPosition());
        motorL1.setTargetPosition(motorL1.getCurrentPosition() + (int) targetCounts);
        motorL2.setTargetPosition(motorL2.getCurrentPosition() + (int) targetCounts);
        motorR1.setTargetPosition(motorR1.getCurrentPosition() + (int) targetCounts);
        motorR2.setTargetPosition(motorR2.getCurrentPosition() + (int) targetCounts);

        setDriveSysMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorL1.setPower(speed * frictionCoefficient);
        motorL2.setPower(speed * frictionCoefficient);
        motorR1.setPower(speed * frictionCoefficient);
        motorR2.setPower(speed * frictionCoefficient);

        while(motorL1.isBusy()&&motorL2.isBusy()&&motorR1.isBusy()&&motorR2.isBusy()){
            curOpMode.idle();
        }

        this.stop();

        DbgLog.msg("motorL1 current position = %d", motorL1.getCurrentPosition());
        setDriveSysMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    private void setDriveSysMode(DcMotor.RunMode runMode){
        motorL1.setMode(runMode);
        motorL2.setMode(runMode);
        motorR1.setMode(runMode);
        motorR2.setMode(runMode);
    }

    @Override
    public void setMaxSpeed(float speed){
        motorL1.setMaxSpeed((int) (motorL1MaxSpeed * speed));
        motorL2.setMaxSpeed((int) (motorL2MaxSpeed * speed));
        motorR1.setMaxSpeed((int) (motorR1MaxSpeed * speed));
        motorR2.setMaxSpeed((int) (motorR2MaxSpeed * speed));
    }
    /*public void driveToDistance(float speed, float direction, double distance){
        double startingPositionL = motorL1.getCurrentPosition();
        double startingPositionR = motorR1.getCurrentPosition();

        double targetPosition =(distance / wheelValues[1]) * 1120;

        while(((motorL1.getCurrentPosition()-startingPositionL)<targetPosition) && ((motorR.getCurrentPosition()-startingPositionR)<targetPosition)){
            drive(speed, direction);
        }
        motorR.setPower(0);
        motorL.setPower(0);
    }*/
}
