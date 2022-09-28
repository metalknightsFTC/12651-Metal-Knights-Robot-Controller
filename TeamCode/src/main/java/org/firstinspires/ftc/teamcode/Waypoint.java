package org.firstinspires.ftc.teamcode;

public class Waypoint {

    public float xStart;
    public float zStart;
    public float headingStart;

    public float xEnd;
    public float zEnd;
    public float headingEnd;

    float x;
    float heading;
    float z;

    public  Waypoint(float x, float heading, float z){
        this.x = x;
        this.heading = heading;
        this.z = z;
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
