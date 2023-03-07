package org.firstinspires.ftc.teamcode.Vectors;
import org.firstinspires.ftc.teamcode.Vectors.*;

public class Vector4 {

    public float x,y,z,p;

    public Vector4 (float x, float y, float z,float p)
    {
        this.x = x;
        this.y = y;
        this.z = z;
	    this.p = p;
    }

    public Vector4 ()
    {
        this.x = 0;
        this.y = 0;
        this.z = 0;
        this.p = 0;
    }

    public Vector4 Add(Vector3 vector3){
        x += vector3.x;
        y += vector3.y;
        z += vector3.z;
        return  this;
    }

    public  Vector4 Subtract(Vector3 vector3){
        x -= vector3.x;
        y -= vector3.y;
        z -= vector3.z;
        return  this;
    }

    public  Vector4 Inverse(){
        x = -x;
        y = -y;
        z = -z;
        return  this;
    }

    public  Vector4 Multiply(float multiplier){
        x *= multiplier;
        y *= multiplier;
        z *= multiplier;
        return this;
    }

    public static Vector4 zero(){return new Vector4(0,0,0,0);}

}
