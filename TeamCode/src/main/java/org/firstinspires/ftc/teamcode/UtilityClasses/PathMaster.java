package org.firstinspires.ftc.teamcode.UtilityClasses;
import org.firstinspires.ftc.teamcode.Vectors.*;

public class PathMaster {

    Vector4[] changesInPosition;

    public  Vector4 NextPoint(int index)
    {
        return  changesInPosition[index];
    }

    public int totalPoints()
    {
        return changesInPosition.length;
    }

    //p acts as operation indicator
    //if p is 2 then lift to lvl 0
    //if p is 3 then lift to lvl 1
    //if p is 4 then lift to lvl 2
    //if p is 5 then lift to lvl -1
    //if p is 6 then open claw
    //if p is 7 then close claw

    public void InitializePathSet1(int pathCode)
    {
        switch (pathCode)
        {
            case 0:
                changesInPosition = new Vector4[4];
                changesInPosition[0] = new Vector4(3.5f,0,0,.3f);
                changesInPosition[1] = new Vector4(0f,0,-16,.4f);
                changesInPosition[2] = new Vector4(4f,0,0,.4f);
                changesInPosition[3] = new Vector4(0f,0,-2,.4f);
                break;
            case 1:
                changesInPosition = new Vector4[7];
            default:
                break;
        }
    }

    public void InitializePathSet2(int pathCode)
    {
        switch (pathCode)
        {
            case 0:
                changesInPosition = new Vector4[2];
                changesInPosition[0] = new Vector4(32f,0,0,.3f);
                changesInPosition[1] = new Vector4(0f,0,40,.4f);
                break;
            case 1:
                changesInPosition = new Vector4[2];
            default:
                break;
        }
    }

}
