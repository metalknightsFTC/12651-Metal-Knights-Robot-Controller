package org.firstinspires.ftc.teamcode;

public class AutoPilot {
    boolean apActive = false;

    OdometryControl driveControlSystems;

    int selectedPath;

    Path[] paths;

    public  void  ActivateAP(int pathToExecute, OdometryControl odometryControl){
        selectedPath = pathToExecute;
        apActive = true;
        driveControlSystems = odometryControl;
        Update();
    }

    public  void  DeactivateAp(){
        apActive = false;
    }

    public  void  Update(){
        while (apActive){

        }
    }

}
