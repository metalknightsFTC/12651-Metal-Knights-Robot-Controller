package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.Vectors.*;

public class PathMaster {

    Vector4[] changesInPosition;

    public  Vector4 NextPoint(int index){
        return  changesInPosition[index];
    }

    public int points()
    {
        return changesInPosition.length;
    }

    public void InitializePath(int pathCode){
        switch (pathCode){
            case 0:
                changesInPosition = new Vector4[3];
                changesInPosition[0] = new Vector4(3.5f,0,0,.25f);
                break;
            case 1:
                changesInPosition = new Vector4[1];
            default:
                break;
        }
    }

}
