//imports the package so that all the code can be used
package org.firstinspires.ftc.teamcode;

//all the import
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

//allows me to use the package code
import org.firstinspires.ftc.robotcontroller.external.samples.BasicOpMode_Iterative;
import org.firstinspires.ftc.teamcode.Control.BallisticMotionProfile;
import org.firstinspires.ftc.teamcode.Robot.RobotLinearOpMode;

//imports all of the math function
import static java.lang.Math.*;
import static org.firstinspires.ftc.teamcode.Robot.RobotLinearOpMode.MOVEMENT_DIRECTION.FORWARD;
import static org.firstinspires.ftc.teamcode.Robot.RobotLinearOpMode.MOVEMENT_DIRECTION.STRAFE;

//name that appears on the phone and the group that it is a part of
@Autonomous(name="Rubie Test OpMode", group="Linear Opmode")

//class name that corresponds with the file, it is a linear opMode that I then changed
public class rubieAutoPaths extends LinearOpMode {


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

        robotLinearOpMode.setDriveTrainRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotLinearOpMode.setDriveTrainZeroPowerProperty(DcMotor.ZeroPowerBehavior.BRAKE);

        robotLinearOpMode.setIntakeRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robotLinearOpMode.setIntakeZeroPowerProperty(DcMotor.ZeroPowerBehavior.BRAKE);

        robotLinearOpMode.setArmRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotLinearOpMode.setArmZeroPowerProperty(DcMotor.ZeroPowerBehavior.BRAKE);


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //this is when the robot receives the 'play' command
        waitForStart();
        runtime.reset();
        









//    enum AUTOPATHNAMES {
//    R1FOUNDATIONPARKR, R1FOUNDATIONSTONEPARKR, R1FOUNDATIONSTONESTONE2PARKR, R1FOUNDATIONPARKL, R1FOUNDATIONSTONEPARKL, R1FOUNDATIONSTONESTONE2PARKL,
//    R1PARKL, R1PARKR, R1STONEFOUNDATIONPARKL, R1STONEFOUNDATIONPARKR, R1STONESTONE2FOUNDATIONPARKL, R1STONESTONE2FOUNDATIONPARKR, R1STONEPARKL, R1STONEPARKR,
//    R1STONESTONE2PARKL, R1STONESTONE2PARKR,
//        R2FOUNDATIONPARKR, R2FOUNDATIONSTONEPARKR, R2FOUNDATIONSTONESTONE2PARKR, R2FOUNDATIONPARKL, R2FOUNDATIONSTONEPARKL, R2FOUNDATIONSTONESTONE2PARKL,
//        R2PARKL, R2PARKR, R2STONEFOUNDATIONPARKL, R2STONEFOUNDATIONPARKR, R2STONESTONE2FOUNDATIONPARKL, R2STONESTONE2FOUNDATIONPARKR, R2STONEPARKL, R2STONEPARKR,
//        R2STONESTONE2PARKL, R2STONESTONE2PARKR,
//       NR1FOUNDATIONPARKR,NR1FOUNDATIONSTONEPARKR,NR1FOUNDATIONSTONESTONE2PARKR,NR1FOUNDATIONPARKL,NR1FOUNDATIONSTONEPARKL,NR1FOUNDATIONSTONESTONE2PARKL,
//       NR1PARKL,NR1PARKR,NR1STONEFOUNDATIONPARKL,NR1STONEFOUNDATIONPARKR,NR1STONESTONE2FOUNDATIONPARKL,NR1STONESTONE2FOUNDATIONPARKR,NR1STONEPARKL,NR1STONEPARKR,
//       NR1STONESTONE2PARKL,NR1STON
//                NR2FOUNDATIONPARKR,NR2FOUNDATIONSTONEPARKR,NR2FOUNDATIONSTONESTONE2PARKR,NR2FOUNDATIONPARKL,NR2FOUNDATIONSTONEPARKL,NR2FOUNDATIONSTONESTONE2PARKL,
//                NR2PARKL,NR2PARKR,NR2STONEFOUNDATIONPARKL,NR2STONEFOUNDATIONPARKR,NR2STONESTONE2FOUNDATIONPARKL,NR2STONESTONE2FOUNDATIONPARKR,NR2STONEPARKL,NR2STONEPARKR,
//                NR2STONESTONE2PARKL,NR2STONESTONE2PARKR,
//
//    }


/* foundation from start (starting facing the wall)
        back up ~2 feet
        latch
        strafe to right until foundation hits wall
        start next thing
 */

        /* foundation from stone ()*/

    }

}