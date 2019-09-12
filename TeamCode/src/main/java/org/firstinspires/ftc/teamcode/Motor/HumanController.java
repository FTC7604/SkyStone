package org.firstinspires.ftc.teamcode.Motor;

/*A short little class that I wrote that better transforms the numeric imputs of a human,
to a more humanistic value.
 */
public class HumanController {

    public static final double humanController(double input) {
        double output = 0;

        if (input > .075) {
            output = .5 * Math.log10(input - 0.05) + 1.012;
        } else if (input >= -.075 && input <= .075) {
            output = 500 * input * input * input;
        } else if (input < -.075) {
            output = -.5 * Math.log10(-input - 0.05) - 1.012;
        }

        return output;
    }

    //Neverest Claims that the motor can freeload 160 rpm.
    //given that the motors will in all likelihood be understress, I'm going to assume that the robot can do at most 10 rpm

}