package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Enums.Motor;
import org.firstinspires.ftc.teamcode.Enums.SelectedDrive;
//import com.qualcomm.robotcore.util.Hardware;

public class OdometryControl {

    float omniBaseWidth = 266.7f; //266.7f mm 10.5in
    float omniBaseLength = 114.3f; //114.3f mm 4.5in
    //229mm tolerance
    float diameter = 50f; //50mm 1.96in
    int cpr = 8192;
    float c = (float)((2 * Math.PI * (diameter / 2)));
    Gamepad lGpad1;
    HardwareMap lHardwareMap;
    Position robotPosition;
    DriveTrainCode driveTrainCode;

    DcMotorEx right;
    DcMotorEx left;
    DcMotorEx rear;

    float heading = 0f;

    float currentLeftEncoderRotation = 0;
    float currentRightEncoderRotation = 0;
    float currentRearEncoderRotation = 0;
    float leftEncoderRotation = 0;
    float rightEncoderRotation = 0;
    float rearEncoderRotation = 0;


    public  OdometryControl(DcMotorEx right1, DcMotorEx left1, DcMotorEx rear1, HardwareMap hardwareMap, Gamepad gpad){
        robotPosition = new Position();
        lGpad1 = gpad;
        right = right1;
        left = left1;
        rear = rear1;

        lHardwareMap = hardwareMap;
        driveTrainCode = new DriveTrainCode(lGpad1, lHardwareMap);
        rear.setDirection(DcMotorSimple.Direction.REVERSE);
        driveTrainCode.InvertMotorDirection(Motor.frontLeft);
        driveTrainCode.InvertMotorDirection(Motor.backLeft);
        driveTrainCode.InvertMotorDirection(Motor.backRight);
        leftEncoderRotation = left.getCurrentPosition();
        rightEncoderRotation = right.getCurrentPosition();
        rearEncoderRotation = rear.getCurrentPosition();

    }

    public  void  MoveToPoint(Waypoint target){
        double distToTarget = Math.sqrt((Math.pow(target.xEnd, 2) - Math.pow(robotPosition.x, 2)) +
                (Math.pow(target.zEnd, 2) - Math.pow(robotPosition.z, 2)));

        float deltaX = target.xEnd - robotPosition.x;
        float deltaZ = target.zEnd - robotPosition.z;
        float deltaTheta = target.headingEnd - robotPosition.currentHeading;

        SetStickPower(deltaX,deltaZ,deltaTheta);
    }

    public float[] CalculateRobotPosition()
    {

        float deltaX = 0;
        float deltaZ = 0;
        float deltaTheta = 0;

        leftEncoderRotation = currentLeftEncoderRotation;
        rightEncoderRotation = currentRightEncoderRotation;
        rearEncoderRotation = currentRearEncoderRotation;

        currentRightEncoderRotation = right.getCurrentPosition();
        currentRearEncoderRotation = rear.getCurrentPosition();
        currentLeftEncoderRotation = left.getCurrentPosition();

        float deltaLeft = (currentLeftEncoderRotation - leftEncoderRotation)/cpr;
        float deltaRight = (currentRightEncoderRotation - rightEncoderRotation)/cpr;
        float deltaBack = (currentRearEncoderRotation - rearEncoderRotation)/cpr;

        deltaZ = (float) (((((deltaLeft * c) * Math.cos(heading))
                        + ((deltaRight * c) * Math.cos(heading))) / 2f)
                        - (deltaBack * c) * Math.sin(heading));

        deltaX = (float) (((deltaBack * c) * Math.cos(heading)) + ((((deltaLeft * c) * Math.sin(heading))
                        + ((deltaRight * c) * Math.sin(heading))) / 2f));

        deltaTheta = c * (deltaRight - deltaLeft) / .25f;

        return new float[] {deltaX,deltaZ,deltaTheta};
    }


    public void CalculatePosition()
    {

        leftEncoderRotation = currentLeftEncoderRotation;
        rightEncoderRotation = currentRightEncoderRotation;
        rearEncoderRotation = currentRearEncoderRotation;

        currentRightEncoderRotation = right.getCurrentPosition();
        currentRearEncoderRotation = rear.getCurrentPosition();
        currentLeftEncoderRotation = left.getCurrentPosition();

        float dn1 = currentLeftEncoderRotation - leftEncoderRotation;
        float dn2 = currentRightEncoderRotation - rightEncoderRotation;
        float dn3 = currentRearEncoderRotation - rearEncoderRotation;
        float turnL = 1;
        float turnR = 1;



        float deltaTheta = c * ((dn2 - dn1) / omniBaseWidth);
        float deltaZ = c * ((dn1 + dn2) / 2);
        float deltaX = c * ((dn3 - ((dn2 * turnL) + (-dn1 * turnR))) * (omniBaseLength / omniBaseWidth));

        float theta = (float)((robotPosition.currentHeading * (Math.PI/180)) + (deltaTheta / 2.0));
        robotPosition.x += (deltaX * Math.cos(theta)) - (deltaZ * Math.sin(theta));
        robotPosition.z += (deltaX * Math.sin(theta)) + (deltaZ * Math.cos(theta));

        robotPosition.currentHeading += deltaTheta * (180/Math.PI);
        if (robotPosition.currentHeading > 360){
            robotPosition.currentHeading -= 360;
        }
        if (robotPosition.currentHeading < 0){
            robotPosition.currentHeading += 360;
        }
    }

    public void SetStickPower(float x, float z, float turn)
    {
        driveTrainCode.UpdateDriveTrain(SelectedDrive.autonomous, x, z, turn);
    }

}