package org.firstinspires.ftc.teamcode.Control;

//class for the toggle switch.
public class Toggle {
    //toggle is duh, loop is the previous loop
    private boolean toggle;
    private boolean loop;

    //creates the toggle as either any value
    public Toggle(boolean start){toggle = start;}
    public Toggle(){toggle = false;}

    //returns the value of the toggle
    public boolean get(){ return toggle;}
    //indicates wether or not the switch has toggled
    public boolean changed(boolean newValue){
        if(newValue != loop)return true;
        else return false;
    }
    //changes the toggle based on the new data
    public void update(boolean newValue){
        
		if(changed(newValue)){
			if(toggle)toggle = false;
			else if(!toggle)toggle = true;
		}
		
        loop = newValue;
    }
}
