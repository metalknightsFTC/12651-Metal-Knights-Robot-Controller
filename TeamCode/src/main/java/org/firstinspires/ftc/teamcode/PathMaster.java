package org.firstinspires.ftc.teamcode;

public class PathMaster {

    public Vector3[] changesInPosition;

    public void InitializePath(int pathCode){
        switch (pathCode){
            case 0:
                changesInPosition = new Vector3[3];
                break;
            case 1:
                changesInPosition = new Vector3[3];
                break;
            case 2:
                changesInPosition = new Vector3[3];
                break;
            case 3:
                changesInPosition = new Vector3[3];
                break;
            case 4:
                changesInPosition = new Vector3[3];
                break;
            case 5:
                changesInPosition = new Vector3[3];
                break;
            default:
                break;
        }
    }

}
