//imports the package so that all the code can be used
package org.firstinspires.ftc.teamcode;

//all the import

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Control.BallisticMotionProfile;
import org.firstinspires.ftc.teamcode.Control.EverHit;
import org.firstinspires.ftc.teamcode.Control.HumanController;
import org.firstinspires.ftc.teamcode.Control.Toggle;
import org.firstinspires.ftc.teamcode.Robot.RobotLinearOpMode;

import static java.lang.Math.abs;

//allows me to use the package code
//imports all of the math function

//name that appears on the phone and the group that it is a part of
@TeleOp(name = "Test OpMode", group = "Linear Opmode")

//class name that corresponds with the file, it is a linear opMode that I then changed
public class testOpMode extends LinearOpMode {


    /*Mini Lesson:
    Public - Can be accessed from anywhere
    Private - Can be accessed from within the class
    *Nothing* - Can be accessed from within the package (package private)
    Proteccted - Subclasses can access the object (think extends)
     */

    //so these exist so that I do not get confused, not really necessary
    double[] driveTrainController = new double[3];

    /*Mini Lesson:
    Int - Whole numbers
    Long - Long whole numbers
    Float - Whole numbers but hexidecimal(base 16)
    Double - Floating point (decimal values)
    Char - Character, the basis of a word
    Boolean - True/False
     */
    double intakePower = 0;
    double armPower = 0;
    double liftPower = 0;
    //sensor values, also exist to make the code cleaner
    double armPosition = 0;
    int liftPosition;
    boolean blockIntakeTouchSensor;
    RobotLinearOpMode robotLinearOpMode;
    Toggle latchToggle;
    Toggle grabberToggle;
    EverHit blockEverInIntake;
    Toggle driveMode;
    double topArmEncoder = 2700;
    double bottomArmEncoder = -3900;
    HumanController humanController = new HumanController(0.1, 1);
    //the amount of time that the program has run
    private ElapsedTime runtime = new ElapsedTime();

    //this is the loop that repeats until the end of teleOp.
    @Override
    public void runOpMode() {
        robotLinearOpMode = new RobotLinearOpMode(this);

        robotLinearOpMode.setDriveTrainRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robotLinearOpMode.setDriveTrainZeroPowerProperty(DcMotor.ZeroPowerBehavior.BRAKE);

        robotLinearOpMode.setIntakeRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robotLinearOpMode.setIntakeZeroPowerProperty(DcMotor.ZeroPowerBehavior.BRAKE);

        robotLinearOpMode.setArmRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotLinearOpMode.setArmZeroPowerProperty(DcMotor.ZeroPowerBehavior.BRAKE);

        robotLinearOpMode.setLiftRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotLinearOpMode.setLiftZeroPowerProperty(DcMotor.ZeroPowerBehavior.BRAKE);

        latchToggle = new Toggle(false);
        grabberToggle = new Toggle(false);
        driveMode = new Toggle(false);
        blockEverInIntake = new EverHit();

        //BallisticMotionProfile liftProfile = new BallisticMotionProfile(0, 20000, 400, 0.05, 1, .5);
        BallisticMotionProfile armProfile = new BallisticMotionProfile(bottomArmEncoder, topArmEncoder, 100, 0, 1, .5);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //this is when the robot receives the 'play' command
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            //sets up the condidtion for the drivetrain
            driveMode.update(gamepad1.right_bumper);

            if(driveMode.get() == false) {
                driveTrainController[1] = humanController.linearDriveProfile(((-gamepad1.right_stick_y) * (abs(-gamepad1.right_stick_y)) + ((-gamepad1.left_stick_y) * (abs(-gamepad1.left_stick_y)))) / 2);
                driveTrainController[0] = humanController.linearDriveProfile(-(((-gamepad1.right_stick_x) * (abs(-gamepad1.right_stick_x)) + ((-gamepad1.left_stick_x) * (abs(-gamepad1.left_stick_x)))) / 2));
                driveTrainController[2] = humanController.linearDriveProfile(((-gamepad1.right_stick_y) - (-gamepad1.left_stick_y)) / 2);
            }else{
                driveTrainController[1] = humanController.linearDriveProfile(-gamepad1.left_stick_y);
                driveTrainController[0] = humanController.linearDriveProfile(gamepad1.left_stick_x);
                driveTrainController[2] = humanController.linearDriveProfile(-gamepad1.right_stick_x);
            }

            if(gamepad1.left_bumper){
                driveTrainController[1] /= 2;
                driveTrainController[0] /= 2;
                driveTrainController[2] /= 2;
            }
            //increments the intake power
            intakePower = gamepad2.right_trigger - gamepad2.left_trigger;

            //sets the arm power
            armPower = armProfile.limitWithoutAccel(robotLinearOpMode.getArmEncoder(), gamepad2.right_stick_y);

            //liftPower = liftProfile.V2limitWithAccel(robotLinearOpMode.getLiftEncoder(),-gamepad2.left_stick_y);
            liftPower = -gamepad2.left_stick_y/2;

            latchToggle.update(gamepad2.x);
            grabberToggle.update(gamepad2.y);

            blockEverInIntake.update(robotLinearOpMode.blockInIntake());

            if (grabberToggle.get()) robotLinearOpMode.openGrabber();
            else robotLinearOpMode.closeGrabber();

            if (latchToggle.get()) robotLinearOpMode.openLatch();
            else robotLinearOpMode.closeLatch();

            if (gamepad2.a) blockEverInIntake.reset();

            //sends this to the motors.
            robotLinearOpMode.mecPowerDrive(driveTrainController);
            robotLinearOpMode.setIntakePower(intakePower);
            robotLinearOpMode.setArmPower(armPower);
            robotLinearOpMode.setLiftPower(liftPower);

            //gets the position arm encoder
            armPosition = robotLinearOpMode.getArmEncoder();
            liftPosition = robotLinearOpMode.getLiftEncoder();
            blockIntakeTouchSensor = robotLinearOpMode.blockInIntake();

            sendTelemetry();
        }
    }

    void sendTelemetry(){
        //now sends it too teleOp
        telemetry.addData("Intake Movement: ", intakePower);
        telemetry.addData("Arm Movement: ", armPower);
        telemetry.addData("Lift Movement: ", liftPower);

        telemetry.addData("Arm Position: ", armPosition);
        telemetry.addData("Lift Position: ", liftPosition);
        telemetry.addData("Intake Touch Boolean: ", blockEverInIntake.get());
        telemetry.update();
    }
}

