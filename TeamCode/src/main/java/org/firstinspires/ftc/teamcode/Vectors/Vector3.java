package org.firstinspires.ftc.teamcode.Vectors;
import org.firstinspires.ftc.teamcode.Vectors.*;

public class Vector3 {

    public float x,y,z;

    public Vector3 (float x, float y, float z)
    {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public Vector3 Add(Vector3 vector3){
        x += vector3.x;
        y += vector3.y;
        z += vector3.z;
        return  this;
    }

    public  Vector3 Subtract(Vector3 vector3){
        x -= vector3.x;
        y -= vector3.y;
        z -= vector3.z;
        return  this;
    }

    public  Vector3 Inverse(){
        x = -x;
        y = -y;
        z = -z;
        return  this;
    }

    public  Vector3 Multiply(float multiplier){
        x *= multiplier;
        y *= multiplier;
        z *= multiplier;
        return this;
    }

    public static Vector3 zero(){
        return new Vector3(0,0,0);
    }

}
