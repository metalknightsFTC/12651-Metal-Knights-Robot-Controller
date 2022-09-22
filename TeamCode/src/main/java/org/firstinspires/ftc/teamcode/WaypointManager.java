package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Enums.StartPoint;
//import com.qualcomm.robotcore.util.Hardware;

public class WaypointManager {

    public  Vector3[] waypoints;

    public Vector3[][] grid;

    public Vector3[] activePath;

    float robotRadius = 14.0f;

    public  int currentWaypoint = 0;
//Red left == 0; red Right == 2; blue left == 1; blue right == 3;
    public  WaypointManager(StartPoint startPoint)
    {
        if (startPoint == StartPoint.redLeft)
        {
            SetupPath0(3);
        }else if (startPoint == StartPoint.blueRight)
        {
            SetupPath1(3);
        } else if (startPoint == StartPoint.redRight)
        {
            SetupPath2(3);
        } else if (startPoint == StartPoint.blueLeft)
        {
            SetupPath3(3);
        }

    }

    public void SetupPath0(int length)
    {
        waypoints = new Vector3[length];
    }

    public void SetupPath1(int length)
    {
        waypoints = new Vector3[length];
    }

    public void SetupPath2(int length)
    {
        waypoints = new Vector3[length];

    }

    public void SetupPath3(int length)
    {
        waypoints = new Vector3[length];
    }

    public void initGrid(){
        grid = new Vector3[12][12];

        for(int x = 0; x < 12; x++)
        {
            for(int z = 0; z < 12; z++)
            {
                grid[x][z] = new Vector3((x * 12) + robotRadius,0,(z * 12) + robotRadius);
            }
        }
    }

}
