package org.firstinspires.ftc.teamcode.Vectors;

public class Vector2 {


    public float x,z;

    public Vector2 (float x, float z)
    {
        this.x = x;
        this.z = z;
    }

    public Vector2 Add(Vector2 vector2){
        x += vector2.x;
        z += vector2.z;
        return  this;
    }

    public  Vector2 Subtract(Vector2 vector2){
        x -= vector2.x;
        z -= vector2.z;
        return  this;
    }

    public  Vector2 Inverse(){
        x = -x;
        z = -z;
        return  this;
    }

    public  Vector2 Multiply(float multiplier){
        x *= multiplier;
        z *= multiplier;
        return this;
    }

    public static Vector2 zero(){
        return new Vector2(0,0);
    }

    public String toString()
    {
        return "X: " + x + ", Z: " + z;
    }

}
