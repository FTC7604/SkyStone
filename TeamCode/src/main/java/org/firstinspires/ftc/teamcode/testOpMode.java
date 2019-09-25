//imports the package so that all the code can be used
package org.firstinspires.ftc.teamcode;

//all the import
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

//allows me to use the package code
import org.firstinspires.ftc.robotcontroller.external.samples.BasicOpMode_Iterative;
import org.firstinspires.ftc.teamcode.Control.BallisticMotionProfile;
import org.firstinspires.ftc.teamcode.Motor.HumanController;
import org.firstinspires.ftc.teamcode.Robot.RobotLinearOpMode;

//imports all of the math function
import static java.lang.Math.*;

//name that appears on the phone and the group that it is a part of
@TeleOp(name="Test OpMode", group="Linear Opmode")

//class name that corresponds with the file, it is a linear opMode that I then changed
public class testOpMode extends LinearOpMode {


    /*Mini Lesson:
    Public - Can be accessed from anywhere
    Private - Can be accessed from within the class
    *Nothing* - Can be accessed from within the package (package private)
    Proteccted - Subclasses can access the object (think extends)
     */

    //the amount of time that the program has run
    private ElapsedTime runtime = new ElapsedTime();

    /*Mini Lesson:
    Int - Whole numbers
    Long - Long whole numbers
    Float - Whole numbers but hexidecimal(base 16)
    Double - Floating point (decimal values)
    Char - Character, the basis of a word
    Boolean - True/False
     */

    //so these exist so that I do not get confused, not really necessary
    double[] driveTrainController = new double[3];
    double intakePower = 0;
    double armPower = 0;
    double liftPower = 0;

    //sensor values, also exist to make the code cleaner
    double armPosition = 0;
    boolean blockIntakeTouchSensor = true;

    //this is the loop that repeats until the end of teleOp.
    @Override
    public void runOpMode() {

        RobotLinearOpMode robotLinearOpMode = new RobotLinearOpMode(this);

        robotLinearOpMode.setDriveTrainRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robotLinearOpMode.setDriveTrainZeroPowerProperty(DcMotor.ZeroPowerBehavior.BRAKE);

        robotLinearOpMode.setIntakeRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robotLinearOpMode.setIntakeZeroPowerProperty(DcMotor.ZeroPowerBehavior.BRAKE);

        robotLinearOpMode.setArmRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotLinearOpMode.setArmZeroPowerProperty(DcMotor.ZeroPowerBehavior.BRAKE);


        BallisticMotionProfile liftProfile = new BallisticMotionProfile(0,-20000, 2000, 0.05, 1, 0.5);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //this is when the robot receives the 'play' command
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            //sets up the condidtion for the drivetrain
            driveTrainController[1] = HumanController.humanController(((-gamepad1.right_stick_y)*(abs(-gamepad1.right_stick_y))+((-gamepad1.left_stick_y)*(abs(-gamepad1.left_stick_y))))/2);
            driveTrainController[0] = - HumanController.humanController(((-gamepad1.right_stick_x)*(abs(-gamepad1.right_stick_x))+((-gamepad1.left_stick_x)*(abs(-gamepad1.left_stick_x))))/2);
            driveTrainController[2] = HumanController.humanController(((-gamepad1.right_stick_y)-(-gamepad1.left_stick_y))/2);

            //increments the intake power
            intakePower = gamepad2.right_trigger - gamepad2.left_trigger;

            //sets the arm power
            armPower = gamepad2.right_stick_y;

            liftPower = liftProfile.V2limitWithAccel(robotLinearOpMode.getLiftEncoder(),-gamepad2.left_stick_y);

            //sends this to the motors.
            robotLinearOpMode.mecPowerDrive(driveTrainController);
            robotLinearOpMode.setIntakePower(intakePower);
            robotLinearOpMode.setArmPower(armPower);
            robotLinearOpMode.setLiftPower(liftPower);

            //gets the position arm encoder
            armPosition = robotLinearOpMode.getArmEncoder();
            int liftPosition = robotLinearOpMode.getLiftEncoder();
            blockIntakeTouchSensor = robotLinearOpMode.blockIntakeTouchSensorIsPressed();

            //now sends it too teleOp
            telemetry.addData("Intake Movement: ", intakePower);
            telemetry.addData("Arm Position: ", armPosition);
            telemetry.addData("Lift Position: ", liftPosition);
            telemetry.addData("Intake Touch Boolean: ", blockIntakeTouchSensor);
            telemetry.update();

        }
    }
}

