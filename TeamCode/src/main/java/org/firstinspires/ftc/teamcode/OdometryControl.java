package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Enums.Motor;
import org.firstinspires.ftc.teamcode.Enums.SelectedDrive;
public class OdometryControl {

    float omniBaseWidth = 8.5f; //266.7f mm 10.5in
    float omniBaseLength = 6.5f; //114.3f mm 4.5in
    //229mm tolerance
    float diameter = 1.96f; //50mm 1.96in
    int cpr = 8192;
    double c = 6.1575216;//(Math.PI) * (diameter / 2);//6.1575216
    Position robotPosition;

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
    public  OdometryControl(DcMotorEx gRight, DcMotorEx gLeft, DcMotorEx gRear,Vector3 start)
    {
        robotPosition = new Position(start.x,start.y,start.z);

        right = gRight;
        left = gLeft;
        rear = gRear;

        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //left.setDirection(DcMotorSimple.Direction.REVERSE);
        right.setDirection(DcMotorSimple.Direction.REVERSE);

        rear.setDirection(DcMotorSimple.Direction.REVERSE);

        currentLeftEncoderRotation = left.getCurrentPosition();
        currentRightEncoderRotation = right.getCurrentPosition();
        currentRearEncoderRotation = rear.getCurrentPosition();
    }
    //endregion

    public  Vector3  MoveToPoint(Vector3 target)
    {
        float deltaX = target.x - robotPosition.x;
        float deltaZ = target.z - robotPosition.z;
        float deltaTheta = target.y - robotPosition.currentHeading;

        if(deltaX < -1){
            deltaX = -1;
        }else if(deltaX > 1){
            deltaX = 1;
        }

        if(deltaZ < -1){
            deltaZ = -1;
        }else if(deltaZ > 1){
            deltaZ = 1;
        }

        if(deltaTheta < -1){
            deltaTheta = -1;
        }else if(deltaTheta>1){
            deltaTheta = 1;
        }

        return  new Vector3(deltaX * .2f, deltaTheta * .2f, deltaZ * .2f);
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

        deltaTheta = ((c * (deltaLeft - deltaRight)) / omniBaseWidth)*(180/Math.PI);

        if (robotPosition.currentHeading > 360){
            robotPosition.currentHeading -= 360;
        }
        if (robotPosition.currentHeading < 0){
            robotPosition.currentHeading += 360;
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


}