package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.Vectors.*;

public class PathMaster {

    Vector3[] changesInPosition;

    public void InitializePath(int pathCode){
        switch (pathCode){
            case 0:
                changesInPosition = new Vector3[3];
                changesInPosition[0] = new Vector3(0,0,0);
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
