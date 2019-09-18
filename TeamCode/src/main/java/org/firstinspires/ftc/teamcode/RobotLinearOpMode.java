package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robot.Robot;

import static java.lang.Thread.sleep;

public class RobotLinearOpMode extends Robot {

    public RobotLinearOpMode(LinearOpMode linearOpMode) {
        super(linearOpMode);
        this.linearOpMode = linearOpMode;
    }

    LinearOpMode linearOpMode;

    //sets the powers on either a power or velocity, and with variables or an array, where 0 is x, 1 is y, and 2 is r
    public void mecPowerDrive(double x, double y, double r){
        leftFrontDriveMotor.setPower(y - x + r);
        leftBackDriveMotor.setPower(y + x + r);
        rightFrontDriveMotor.setPower(y + x - r);
        rightBackDriveMotor.setPower(y - x - r);
    }
    public void mecPowerDrive(double[] controller){
        leftFrontDriveMotor.setPower(controller[1] - controller[0] + controller[2]);
        leftBackDriveMotor.setPower(controller[1] + controller[0] + controller[2]);
        rightFrontDriveMotor.setPower(controller[1] + controller[0] - controller[2]);
        rightBackDriveMotor.setPower(controller[1] - controller[0] - controller[2]);
    }
    public void mecVelocityDrive(double x, double y, double r){
        leftFrontDriveMotor.setVelocity(y - x + r);
        leftBackDriveMotor.setVelocity(y + x + r);
        rightFrontDriveMotor.setVelocity(y + x - r);
        rightBackDriveMotor.setVelocity(y - x - r);
    }
    public void mecVelocityDrive(double[] controller){
        leftFrontDriveMotor.setVelocity(controller[1] - controller[0] + controller[2]);
        leftBackDriveMotor.setVelocity(controller[1] + controller[0] + controller[2]);
        rightFrontDriveMotor.setVelocity(controller[1] + controller[0] - controller[2]);
        rightBackDriveMotor.setVelocity(controller[1] - controller[0] - controller[2]);
    }

    public void setIntakePower(double intakePower){
        rightIntakeMotor.setPower(intakePower);
        leftIntakeMotor.setPower(intakePower);
    }
    public void setIntakeVelocity(double intakeVelocity){
        rightIntakeMotor.setVelocity(intakeVelocity);
        leftIntakeMotor.setVelocity(intakeVelocity);
    }

    public double getAverageDriveTrainEncoder(){
        double averageEncoderPosition = leftFrontDriveMotor.getCurrentPosition();
        averageEncoderPosition += leftBackDriveMotor.getCurrentPosition();
        averageEncoderPosition += rightFrontDriveMotor.getCurrentPosition();
        averageEncoderPosition += rightBackDriveMotor.getCurrentPosition();

        return averageEncoderPosition/4;
    }

    public void turnByDegree(double degree, BallisticMotionProfile TurnProfile){
        initIMU();

        double currentAngle;
        double adjustedMotorPower;

        //gets the current angle from the IMU
        double startAngle = getRev2IMUAngle()[2];

        //calculates the angles the angle that it needs to be at, at the end
        double neededAngle = startAngle + degree;
        currentAngle = startAngle;

        //the loop that will run until the needed Angle is acheived
        if(currentAngle < neededAngle) {
            while ((currentAngle < neededAngle && linearOpMode.opModeIsActive())) {//will run until we get there
                currentAngle = getRev2IMUAngle()[2];////IMU something

                adjustedMotorPower = TurnProfile.RunToPositionWithAccel(startAngle, currentAngle, neededAngle);//get a rotation

                mecPowerDrive(0,0,adjustedMotorPower);
            }
        }
        else if(currentAngle > neededAngle && linearOpMode.opModeIsActive()){
            while ((currentAngle > neededAngle)) {//will run until we get there
                currentAngle = getRev2IMUAngle()[2];////IMU something

                adjustedMotorPower = TurnProfile.RunToPositionWithAccel(startAngle, currentAngle, neededAngle);//get a rotation

                mecPowerDrive(0,0,adjustedMotorPower);
            }
        }
    }

    void stopMotorsAndWait(double seconds){
        mecPowerDrive(0,0,0);
        try {
            sleep((int)(seconds * 1000));
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    private void moveByInches(double inches, BallisticMotionProfile DriveProfile){
        double startPosition = getAverageDriveTrainEncoder();
        double currentPosition;

        double neededInches = inches;//change this

        double neededPosition = startPosition + (neededInches*4000/69);
        double adjustedMotorPower;

        setDriveTrainRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveTrainRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if(startPosition < neededPosition) {
            while ((getAverageDriveTrainEncoder() < neededPosition) && linearOpMode.opModeIsActive()) {
                currentPosition = getAverageDriveTrainEncoder();

                adjustedMotorPower = DriveProfile.RunToPositionWithAccel(startPosition, currentPosition, neededPosition);

                mecPowerDrive(0,adjustedMotorPower,0);
            }
        }
        else if (startPosition > neededPosition) {
            while ((getAverageDriveTrainEncoder() > neededPosition) && linearOpMode.opModeIsActive()) {
                currentPosition = getAverageDriveTrainEncoder();

                adjustedMotorPower = DriveProfile.RunToPositionWithAccel(startPosition, currentPosition, neededPosition);

                mecPowerDrive(0,adjustedMotorPower,0);
            }
        }
        else {
            //no where to move in here.
        }
    }

}
