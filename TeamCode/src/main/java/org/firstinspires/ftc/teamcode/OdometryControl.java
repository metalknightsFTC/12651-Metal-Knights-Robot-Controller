package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Enums.Motor;
import org.firstinspires.ftc.teamcode.Enums.SelectedDrive;
//import com.qualcomm.robotcore.util.Hardware;

public class OdometryControl {

    float omniBaseWidth = 9.7f; //266.7f mm 10.5in
    float omniBaseLength = 3.65f; //114.3f mm 4.5in
    //229mm tolerance
    float diameter = 1.96f; //50mm 1.96in
    int cpr = 8192;
    double c = (float) (2 * Math.PI) * (diameter / 2);//6.1575216
    Gamepad lGpad1;
    HardwareMap lHardwareMap;
    Position robotPosition;
    DriveTrainCode driveTrainCode;

    DcMotorEx right;
    DcMotorEx left;
    DcMotorEx rear;

    float heading = 0f;

    double currentLeftEncoderRotation = 0;
    double currentRightEncoderRotation = 0;
    double currentRearEncoderRotation = 0;
    double leftEncoderRotation = 0;
    double rightEncoderRotation = 0;
    double rearEncoderRotation = 0;

    //region Constructor
    public  OdometryControl(DcMotorEx right1, DcMotorEx left1, DcMotorEx rear1, HardwareMap hardwareMap, Gamepad gpad)
    {
        robotPosition = new Position();
        lGpad1 = gpad;
        right = right1;
        left = left1;
        rear = rear1;

        lHardwareMap = hardwareMap;/*
        driveTrainCode = new DriveTrainCode(lGpad1, lHardwareMap);
        driveTrainCode.InvertMotorDirection(Motor.frontLeft);
        driveTrainCode.InvertMotorDirection(Motor.backLeft);
        driveTrainCode.InvertMotorDirection(Motor.backRight);*/

        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        currentLeftEncoderRotation = left.getCurrentPosition();
        currentRightEncoderRotation = right.getCurrentPosition();
        currentRearEncoderRotation = rear.getCurrentPosition();
    }
    //endregion

    public  void  MoveToPoint(Waypoint target)
    {
        double distToTarget = Math.sqrt((Math.pow(target.xEnd, 2) - Math.pow(robotPosition.x, 2)) +
                (Math.pow(target.zEnd, 2) - Math.pow(robotPosition.z, 2)));

        float deltaX = target.xEnd - robotPosition.x;
        float deltaZ = target.zEnd - robotPosition.z;
        float deltaTheta = target.headingEnd - robotPosition.currentHeading;

        SetStickPower(deltaX,deltaZ,deltaTheta);
    }

    //region current position calculator algorithm
    public double[] CalculateRobotPosition()
    {

        heading = robotPosition.currentHeading;

        double deltaX = 0;
        double deltaZ = 0;
        double deltaTheta = 0;

        leftEncoderRotation = currentLeftEncoderRotation;
        rightEncoderRotation = currentRightEncoderRotation;
        rearEncoderRotation = currentRearEncoderRotation;

        currentRightEncoderRotation = right.getCurrentPosition();
        currentRearEncoderRotation = rear.getCurrentPosition();
        currentLeftEncoderRotation = left.getCurrentPosition();

        double deltaLeft = (currentLeftEncoderRotation - leftEncoderRotation)/cpr;
        double deltaRight = (currentRightEncoderRotation - rightEncoderRotation)/cpr;
        double deltaBack = (currentRearEncoderRotation - rearEncoderRotation)/cpr;

        deltaZ = (((((deltaLeft * c) * Math.cos(heading))
                        + ((deltaRight * c) * Math.cos(heading))) / 2f)
                        - (deltaBack * c) * Math.sin(heading));

        deltaX = (((deltaBack * c) * Math.cos(heading)) + ((((deltaLeft * c) * Math.sin(heading))
                        + ((deltaRight * c) * Math.sin(heading))) / 2f));

        deltaTheta = ((c * (deltaRight - deltaLeft)) / omniBaseWidth)*(180/Math.PI);

        if(deltaTheta <= 0.1 || deltaTheta >= -0.1)
        {
            deltaX = 0;
        }

        return new double[] {deltaX,deltaZ,deltaTheta};
    }
    //endregion

    //region old position calculator algorithm
    public void CalculatePosition()
    {

        leftEncoderRotation = currentLeftEncoderRotation;
        rightEncoderRotation = currentRightEncoderRotation;
        rearEncoderRotation = currentRearEncoderRotation;

        currentRightEncoderRotation = right.getCurrentPosition();
        currentRearEncoderRotation = rear.getCurrentPosition();
        currentLeftEncoderRotation = left.getCurrentPosition();

        double dn1 = currentLeftEncoderRotation - leftEncoderRotation;
        double dn2 = currentRightEncoderRotation - rightEncoderRotation;
        double dn3 = currentRearEncoderRotation - rearEncoderRotation;
        float turnL = 1;
        float turnR = 1;



        double deltaTheta = c * ((dn2 - dn1) / omniBaseWidth);
        double deltaZ = c * ((dn1 + dn2) / 2);
        double deltaX = c * ((dn3 - ((dn2 * turnL) + (-dn1 * turnR))) * (omniBaseLength / omniBaseWidth));

        double theta = ((robotPosition.currentHeading * (Math.PI/180)) + (deltaTheta / 2.0));
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

    //endregion

    public void SetStickPower(float x, float z, float turn)
    {
        Vector3 driveDirection = new Vector3(x,turn,z);
        driveTrainCode.UpdateDriveTrain(SelectedDrive.autonomous,driveDirection);
    }

}