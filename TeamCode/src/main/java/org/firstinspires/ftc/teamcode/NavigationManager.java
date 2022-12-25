package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Vectors.Vector2;
import org.firstinspires.ftc.teamcode.Vectors.Vector3;

public class NavigationManager
{
    private DcMotorEx right;
    private DcMotorEx left;
    private DcMotorEx rear;

    public static double targetHeading = 0;

    private DriveTrainCode driveTrainCode;

    private IMUController imu;
    public static int tpr = 8192;//found on REV encoder specs chart
    public static double c = 6.1575216; //Circumference of dead wheels (Math.PI) * (diameter / 2);//6.1575216
    public static float rampDown = 3f;
    double errorMargin = .12f;

    public NavigationManager(HardwareMap hardwareMap,IMUController imu, DriveTrainCode driveTrainCode)
    {
        right = hardwareMap.get(DcMotorEx.class,"right");
        left = hardwareMap.get(DcMotorEx.class,"left");
        rear = hardwareMap.get(DcMotorEx.class,"rear");
        this.imu = imu;
        this.driveTrainCode = driveTrainCode;
        imu.ResetAngle();
        targetHeading = imu.GetAngle();
    }

    //Drive x inches in selected direction at selected speed using odometry to record distance and the internal measuring unit(IMU) to correct for drift
    //slows down after a threshold is reached
    //region Movement code
    //the total amount the robot has moved
    int to = 0;
    double totalMovementX = 0;
    double totalMovementZ = 0;
    public void Move(float X, float Z, float speed)
    {
        //how far the robot has left to move
        double xDist = X;
        double zDist = Z;
        //how far the robot has moved so far
        //previously measured encoder positions
        double rearEncoderRotation;
        double rightEncoderRotation;
        double leftEncoderRotation;
        //currently measured encoder rotation
        double currentLeftEncoderRotation = 0;
        double currentRightEncoderRotation = 0;
        double currentRearEncoderRotation = 0;
        //the distance between the current encoder positions and the previous encoder positions
        double deltaLeft;
        double deltaRight;
        double deltaBack;
        //ticks per revolution
        if(speed > .8f)
        {
            rampDown = 20f;
        }else if(speed > .6f)
        {
            rampDown = 12.5f;
        }else
        {
            rampDown = 2.75f;
        }
        //region checks
        while (xDist < -errorMargin || xDist > errorMargin || zDist < -errorMargin || zDist > errorMargin) {
            //region Odometry Math
            leftEncoderRotation = currentLeftEncoderRotation;
            rightEncoderRotation = currentRightEncoderRotation;
            rearEncoderRotation = currentRearEncoderRotation;

            currentRightEncoderRotation = right.getCurrentPosition();
            currentRearEncoderRotation = rear.getCurrentPosition();
            currentLeftEncoderRotation = left.getCurrentPosition();

            deltaLeft = ((currentLeftEncoderRotation - leftEncoderRotation) / tpr) * c;
            deltaRight = ((currentRightEncoderRotation - rightEncoderRotation) / tpr) * c;
            deltaBack = ((currentRearEncoderRotation - rearEncoderRotation) / tpr) * c;
            //endregion

            totalMovementZ += ((deltaLeft + deltaRight) / 2);
            totalMovementX += deltaBack;

            xDist = X - totalMovementX;
            zDist = Z - totalMovementZ;

            float deltaX = (float) (X - totalMovementX) / rampDown;
            float deltaZ = (float) (Z - totalMovementZ) / rampDown;
            deltaX = Range(deltaX, -1, 1);
            deltaZ = Range(deltaZ, -1, 1);
            float turnMod = (float) imu.AngleDeviation(targetHeading) / 20;

            float x = speed * deltaX;
            float y = -turnMod * speed;
            float z = speed * deltaZ;
            float distX = (float)Math.pow(deltaX,3f);
            float distZ = (float)Math.pow(deltaZ,3f);

            if((distX < .03f && distX > -.03f) && (distZ < .03f && distZ > -.03f))
            {
                break;
            }

            driveTrainCode.UpdateDriveTrain(new Vector3(x, y, z));
        }
        //endregion
        ResetNavigationSystem();
        totalMovementX = 0;
        totalMovementZ = 0;
        driveTrainCode.UpdateDriveTrain(new Vector3(0,0,0));
        to = 0;
    }
    //endregion

    public Vector2 DeltaMovement()
    {
        //how far the robot has left to move
        float xDist = 0;
        float zDist = 0;
        //how far the robot has moved so far
        //previously measured encoder positions
        double rearEncoderRotation;
        double rightEncoderRotation;
        double leftEncoderRotation;
        //currently measured encoder rotation
        double currentLeftEncoderRotation = 0;
        double currentRightEncoderRotation = 0;
        double currentRearEncoderRotation = 0;
        //the distance between the current encoder positions and the previous encoder positions
        double deltaLeft;
        double deltaRight;
        double deltaBack;
        //region checks
        //region Odometry Math
        leftEncoderRotation = currentLeftEncoderRotation;
        rightEncoderRotation = currentRightEncoderRotation;
        rearEncoderRotation = currentRearEncoderRotation;

        currentRightEncoderRotation = right.getCurrentPosition();
        currentRearEncoderRotation = rear.getCurrentPosition();
        currentLeftEncoderRotation = left.getCurrentPosition();

        deltaLeft = ((currentLeftEncoderRotation - leftEncoderRotation) / tpr) * c;
        deltaRight = ((currentRightEncoderRotation - rightEncoderRotation) / tpr) * c;
        deltaBack = ((currentRearEncoderRotation - rearEncoderRotation) / tpr) * c;
        //endregion
        zDist += ((deltaLeft + deltaRight) / 2);
        xDist += deltaBack;
        return new Vector2(xDist,zDist);
    }

    public void ResetNavigationSystem()
    {
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void SnapToHeading(float target, float speed){
        float deltaNC = (float) (imu.GetAngle() - target);
        while (deltaNC > .1f || deltaNC < -.1f)
        {
            deltaNC = (float) (imu.GetAngle()-target);
            float delta = (float) deltaNC / 20;

            driveTrainCode.UpdateDriveTrain(new Vector3(0, speed * delta, 0));
        }
        driveTrainCode.UpdateDriveTrain(new Vector3(0,0,0));
        targetHeading = imu.GetAngle();
        ResetNavigationSystem();
    }

    //region lock input between 2 numbers
    private float Range(float in,float lower, float upper){
        float out = in;
        if(out < lower){
            out = lower;
        }
        if(out > upper){
            out = upper;
        }
        return out;
    }
    //endregion

}