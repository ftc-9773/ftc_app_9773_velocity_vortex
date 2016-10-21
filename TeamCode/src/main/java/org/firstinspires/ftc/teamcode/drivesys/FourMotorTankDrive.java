package org.firstinspires.ftc.teamcode.drivesys;

import com.qualcomm.robotcore.hardware.DcMotor;

public class FourMotorTankDrive extends DriveSystem {
    DcMotor motorL1 = null;
    DcMotor motorL2 = null;
    DcMotor motorR1 = null;
    DcMotor motorR2 = null;
    double frictionCoefficient;
    double maxSpeed;
    double minSpeed;
    Wheel wheel;
    int motorCPR;  // Cycles Per Revolution.  == 1120 for Neverest40

    public FourMotorTankDrive(DcMotor motorL1, DcMotor motorL2, DcMotor motorR1, DcMotor motorR2,
                              double maxSpeed, double minSpeed, double frictionCoefficient,
                              Wheel wheel, int motorCPR){
        this.motorL1 = motorL1;
        this.motorL2 = motorL2;
        this.motorR1 = motorR1;
        this.motorR2 = motorR2;
        this.frictionCoefficient = frictionCoefficient;
        this.maxSpeed = maxSpeed;
        this.minSpeed = minSpeed;
        this.wheel = wheel;
        this.motorCPR = motorCPR;
    }

    @Override
    public void drive(float speed, float direction){
        double left = (speed - direction) * frictionCoefficient;
        double right = (speed + direction) * frictionCoefficient;

        motorL1.setPower(left);
        motorL2.setPower(left);
        motorR1.setPower(right);
        motorR2.setPower(right);
    }

    @Override
    public void lineFollow(double leftSpeed, double rightSpeed) {
        motorL1.setPower(leftSpeed);
        motorL2.setPower(leftSpeed);
        motorR1.setPower(rightSpeed);
        motorR2.setPower(rightSpeed);

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
