package org.firstinspires.ftc.teamcode.Robot;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Control.BallisticMotionProfile;
import org.firstinspires.ftc.teamcode.Control.BetterBalisticProfile;
import org.firstinspires.ftc.teamcode.Control.EverHit;
import org.firstinspires.ftc.teamcode.Control.PID;
import org.firstinspires.ftc.teamcode.PropertiesLoader;

import static java.lang.Math.abs;
import static org.firstinspires.ftc.teamcode.Robot.RobotLinearOpMode.MOVEMENT_DIRECTION.FORWARD;

public class RobotLinearOpMode extends Robot {

    private LinearOpMode linearOpMode;



    //private double encodersToInches = 69/4000;
    private double inchesToEncoders = 4000 / 69; //about 60 encoderd ticks to an inch

    //So im gald i got ur attention // heres why the lift code was broken: the bottom limit was set to 20000 not -20000. negative goes up on the lifter, and so the bottom limit is actually the top
    //public BallisticMotionProfile liftProfile = new BallisticMotionProfile(150, 3300, 1000, 0.15, 1, .6);

    private double topArmEncoder = 2300;//I changed this
    private double bottomArmEncoder = 0;//and this. no underpass

    //I cranked up the decel distance so that it decelerates over a longer distance
    public BallisticMotionProfile armProfile = new BallisticMotionProfile(topArmEncoder, bottomArmEncoder, 1000, 0.2, 1, .6);

    private EverHit blockEverInIntake = new EverHit(false);


    //constructor
    public RobotLinearOpMode(LinearOpMode linearOpMode) {

        //creates the robot so that I can use all of the motors
        super(linearOpMode);

        //needs the linear opmode so that I can use telemetry and opModeIsActive()
        this.linearOpMode = linearOpMode;
    }

    //sets the powers on either a power or velocity, and with variables or an array, where 0 is strafe, 1 is forward, and 2 is rotation
    //if I control it using an array, then it just sends it into an earlier method
    private void mecanumPowerDrive(MOVEMENT_DIRECTION movement_direction, double power) {
        switch (movement_direction) {
            case STRAFE:
                mecanumPowerDrive(power, 0, 0);
                break;
            case FORWARD:
                mecanumPowerDrive(0, power, 0);
                break;
            case ROTATION:
                mecanumPowerDrive(0, 0, power);
                break;
        }
    }

    public void mecanumPowerDrive(double strafe, double forward, double rotation) {
        leftFrontDriveMotor.setPower(forward - strafe + rotation);
        leftBackDriveMotor.setPower(forward + strafe + rotation);
        rightFrontDriveMotor.setPower(forward + strafe - rotation);
        rightBackDriveMotor.setPower(forward - strafe - rotation);
    }

    //compensates for change in center of gravity with arm swinging behind the robot
    public void compensatedMecanumPowerDrive(double strafe, double forward, double rotation, double ratio) {
        leftFrontDriveMotor.setPower(forward - strafe + rotation);
        leftBackDriveMotor.setPower(forward + strafe * ratio + rotation);
        rightFrontDriveMotor.setPower(forward + strafe - rotation);
        rightBackDriveMotor.setPower(forward - strafe * ratio - rotation);
    }

    public void mecanumPowerDrive(double[] controller) {
        mecanumPowerDrive(controller[0], controller[1], controller[2]);
    }

    private void mecVelocityDrive(double strafe, double forward, double rotation) {
        leftFrontDriveMotor.setVelocity(forward - strafe + rotation);
        leftBackDriveMotor.setVelocity(forward + strafe + rotation);
        rightFrontDriveMotor.setVelocity(forward + strafe - rotation);
        rightBackDriveMotor.setVelocity(forward - strafe - rotation);
    }

    public void stopAllMotors() {
        leftFrontDriveMotor.setPower(0);
        leftBackDriveMotor.setPower(0);
        rightFrontDriveMotor.setPower(0);
        rightBackDriveMotor.setPower(0);

        leftIntakeMotor.setPower(0);
        rightIntakeMotor.setPower(0);

        liftMotor.setPower(0);

        armMotor.setPower(0);
    }

    public void mecVelocityDrive(double[] controller) {
        mecVelocityDrive(controller[0], controller[1], controller[2]);
    }

    private double getAverageDriveTrainEncoder(MOVEMENT_DIRECTION movement_direction) {
        switch (movement_direction){
            case FORWARD: return getChangeInDriveTrainEncoder()[0];
            case STRAFE: return getChangeInDriveTrainEncoder()[1];
            case ROTATION: return getChangeInDriveTrainEncoder()[2];
        }

        return 0;
    }

    public double[] getChangeInDriveTrainEncoder() {
        return new double[]{
                (+leftFrontDriveMotor.getCurrentPosition() + leftBackDriveMotor.getCurrentPosition() + rightFrontDriveMotor.getCurrentPosition() + rightBackDriveMotor.getCurrentPosition()) / 4,
                (-leftFrontDriveMotor.getCurrentPosition() + leftBackDriveMotor.getCurrentPosition() + rightFrontDriveMotor.getCurrentPosition() - rightBackDriveMotor.getCurrentPosition()) / 4,
                (+leftFrontDriveMotor.getCurrentPosition() + leftBackDriveMotor.getCurrentPosition() - rightFrontDriveMotor.getCurrentPosition() - rightBackDriveMotor.getCurrentPosition()) / 4,
        };
    }

    private double getChangeInDriveTrainEncoder(MOVEMENT_DIRECTION movement_direction, double averageDriveTrainEncoder) {
        return getAverageDriveTrainEncoder(movement_direction) - averageDriveTrainEncoder;
    }

    //two methods that turn precisely, which Casey made and I don't fully understand
    public void turnByDegree(double degree) {

        BallisticMotionProfile TurnProfile = new BallisticMotionProfile(0, 0, 90, .05, 1, .75);

        double currentAngle;
        double adjustedMotorPower;

        //gets the current angle from the IMU
        double startAngle = getRev2IMUAngle()[2];

        //calculates the angles the angle that it needs to be at, at the end
        double neededAngle = startAngle + degree;
        currentAngle = startAngle;

        //the loop that will run until the needed Angle is acheived
        if (currentAngle < neededAngle) {
            while ((currentAngle < neededAngle && linearOpMode.opModeIsActive())) {//will run until we get there
                currentAngle = getRev2IMUAngle()[2];////IMU something

                adjustedMotorPower = TurnProfile.RunToPositionWithAccel(startAngle, currentAngle, neededAngle);//get a rotation

                mecanumPowerDrive(0, 0, adjustedMotorPower);
            }
        } else if (currentAngle > neededAngle && linearOpMode.opModeIsActive()) {
            while ((currentAngle > neededAngle)) {//will run until we get there
                currentAngle = getRev2IMUAngle()[2];////IMU something

                adjustedMotorPower = TurnProfile.RunToPositionWithAccel(startAngle, currentAngle, neededAngle);//get a rotation

                mecanumPowerDrive(0, 0, adjustedMotorPower);
            }
        }
    }

    public enum MOVEMENT_DIRECTION {
        STRAFE,
        FORWARD,
        ROTATION,
    }

    public void moveByInchesWill(double desiredPositionChangeInInches, MOVEMENT_DIRECTION movement_direction) {
        BetterBalisticProfile betterBalisticProfile = new BetterBalisticProfile(
                .1,
                1440 * 2,
                1440 * 2,
                1,
                getAverageDriveTrainEncoder(movement_direction),
                desiredPositionChangeInInches * inchesToEncoders,
                BetterBalisticProfile.CURVE.PROPORTIONAL
        );

        while (!betterBalisticProfile.isComplete() && linearOpMode.opModeIsActive()) {
            betterBalisticProfile.update(getAverageDriveTrainEncoder(movement_direction));
            mecanumPowerDrive(movement_direction, betterBalisticProfile.getAllVelocity());

            linearOpMode.telemetry.addData("Position:", getAverageDriveTrainEncoder(movement_direction));
            linearOpMode.telemetry.addData("Velocity:", betterBalisticProfile.getAllVelocity());
            linearOpMode.telemetry.addData("State:", betterBalisticProfile.getState());
            linearOpMode.telemetry.update();
        }
    }

    public void moveByInches(double desiredPositionChangeInInches, MOVEMENT_DIRECTION movement_direction) {
        moveByInches(desiredPositionChangeInInches, movement_direction, .05);
    }

    public void moveByInches(double desiredPositionChangeInInches, MOVEMENT_DIRECTION movement_direction, double mp) {

        double currentAverageEncoderValue;
        double desiredPositionChangeInEncoders;
        double adjustedMotorPower;
        double startDriveTrainEncoders;

        BallisticMotionProfile DriveProfile = new BallisticMotionProfile(0, 0, 200, mp, 1, 0.8);

        desiredPositionChangeInEncoders = desiredPositionChangeInInches * inchesToEncoders;

        setDriveTrainRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveTrainRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        startDriveTrainEncoders = getAverageDriveTrainEncoder(movement_direction);

        do {

            currentAverageEncoderValue = getAverageDriveTrainEncoder(movement_direction);

            adjustedMotorPower = DriveProfile.RunToPositionWithAccel(startDriveTrainEncoders, currentAverageEncoderValue, desiredPositionChangeInEncoders);

            mecanumPowerDrive(movement_direction, adjustedMotorPower);

        } while ((abs(desiredPositionChangeInEncoders - currentAverageEncoderValue) > 50) && linearOpMode.opModeIsActive());

        stopAllMotors();
    }

    public void moveArmByEncoder(double desiredPositionChangeInEncoders) {

        double startEncoderValue = getArmEncoder();
        double endEncoderValue = desiredPositionChangeInEncoders + startEncoderValue;

        double currentEncoderValue = 0;
        double adjustedMotorPower = 0;

        do {
            currentEncoderValue = getArmEncoder();

            adjustedMotorPower = armProfile.RunToPositionWithAccel(startEncoderValue, currentEncoderValue, endEncoderValue);

            setArmPower(adjustedMotorPower);

        } while ((abs(desiredPositionChangeInEncoders) > abs(startEncoderValue - currentEncoderValue)) && linearOpMode.opModeIsActive());
    }

    //eliminates residual forces
    public void stopMotorsAndWait(double seconds) {
        mecanumPowerDrive(0, 0, 0);
        linearOpMode.sleep((int) (seconds * 1000));
    }

    //sets the intake power and velocity
    public void setIntakePower(double intakePower) {
        if (intakeIsOpen()) {
            rightIntakeMotor.setPower(intakePower);
            leftIntakeMotor.setPower(-intakePower);
        } else {
            rightIntakeMotor.setPower(intakePower);
            leftIntakeMotor.setPower(intakePower);
        }

    }

    BallisticMotionProfile liftProfile = new BallisticMotionProfile(
            20,
            1300,
            1,
            .05,
            1,
            .8
    );

    public void setLiftPower(double liftPower) {
        PID liftPID = new PID();
        double adjustedliftPower = liftProfile.limitWithoutAccel(liftMotor.getCurrentPosition(), liftPower);
        liftMotor.setPower(adjustedliftPower);
    }

    public int getLiftEncoder() {
        return liftMotor.getCurrentPosition();
    }

    //sets the arm power and velocity
    public void setArmPower(double armPower) {
        armMotor.setPower(armPower);
    }

    public int getArmEncoder() {
        return armMotor.getCurrentPosition();
    }

    //tells us if the arm is on targetEncoder
    public boolean armHasArrived(double targetEncoder) {
        boolean arrivedAtTargetEncoder;

        //if the target encoder position is below the very starting one, and the current position is below that
        //now we make a variable to use later which represents the initial position when doing a runtoposition command
        double initialArmPosition = 0;
        if ((initialArmPosition < targetEncoder) && (targetEncoder < getArmEncoder())) {
            arrivedAtTargetEncoder = true;
        }
        //the exact opposite condidion that yields essentially the same result
        else if ((initialArmPosition > targetEncoder) && (targetEncoder > getArmEncoder())) {
            arrivedAtTargetEncoder = true;
        } else {
            arrivedAtTargetEncoder = false;
        }

        return arrivedAtTargetEncoder;
    }

    public void closeGrabber() {
        blockGrabberServo.setPosition(.7);
    }

    public void openGrabber() {
        blockGrabberServo.setPosition(.4);
    }

    public void dropMarker() {
        markerLatchServo.setPosition(1);
    }

    public void holdMarker() {
        markerLatchServo.setPosition(0);
    }

    public void openLatch() {
        leftLatchServo.setPosition(.5);
        rightLatchServo.setPosition(.45);
    }

    public void closeLatch() {
        leftLatchServo.setPosition(.7);
        rightLatchServo.setPosition(.65);
    }

    public boolean isBlockInIntake() {
        blockEverInIntake.update(blockInIntake());
        return blockEverInIntake.wasEverHit();
    }

    public void blockHasLeftIntake() {
        blockEverInIntake.reset();
    }
}