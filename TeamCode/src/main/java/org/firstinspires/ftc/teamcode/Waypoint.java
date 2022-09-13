package org.firstinspires.ftc.teamcode;

public class Waypoint {

    public float xStart;
    public float zStart;
    public float headingStart;

    public float xEnd;
    public float zEnd;
    public float headingEnd;

    public  Waypoint(float xS,float zS,float hS, float xE,float zE,float hE){
        xStart = xS;
        zStart = zS;
        headingStart = hS;
        xEnd = xE;
        zEnd = zE;
        headingEnd = hE;
    }

    Vector3 GetBezierPosition(float t, Vector3 start, Vector3 end, Vector3 sFwd, Vector3 sEnd)
    {

        Vector3 p0 = start;
        Vector3 p1 = p0.Add(sFwd);
        Vector3 p3 = end;
        Vector3 p2 = p3.Subtract(sEnd.Inverse());

        // here is where the magic happens!
        return p0.Multiply((float)Math.pow(1 - t, 3)).Add(p1.Multiply((float)(3f * Math.pow(1f - t, 2f) * t))).Add(p2.Multiply((float) (3f * (1f - t) * Math.pow(t, 2))).Add(p3.Multiply((float) Math.pow(t, 3f))));
    }

}
