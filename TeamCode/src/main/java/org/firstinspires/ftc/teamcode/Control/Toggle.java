package org.firstinspires.ftc.teamcode.Control;

/**
 * Created by Will McCormick, 2018-2019
 * Logic class for creating a toggle variable, which will toggle on/off on button presses
 * This makes it such that instead of being true when a button is pressed, it will persist until the button is pressed again
 */
public class Toggle {
    //toggle is duh, loop is the previous loop
    private boolean toggle;
    private boolean oldValue;

    //creates the toggle as either any value
    public Toggle(boolean start){toggle = start;}
    public Toggle(){toggle = false;}

    //returns the value of the toggle
    public boolean get(){ return toggle;}
    //indicates wether or not the switch has toggled
    public boolean changed(boolean newValue){
        return newValue != oldValue;
    }
    //changes the toggle based on the new data
    public void update(boolean newValue){
        
		if(changed(newValue) && newValue){
			if(toggle)toggle = false;
			else if(!toggle)toggle = true;
		}
		
        oldValue = newValue;
    }
}
