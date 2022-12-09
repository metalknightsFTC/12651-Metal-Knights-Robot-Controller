package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LiftManager {
    private DcMotor lift;
    private int level;
    private int[] liftHeights = new int[8];
    private int targetRotations = 0;

    public LiftManager (HardwareMap hardwareMapG){
        liftHeights[7] = 4150;
        liftHeights[6] = 2906;
        liftHeights[5] = 1776;
        liftHeights[4] = 640;
        liftHeights[3] = 532;
        liftHeights[2] = 331;
        liftHeights[1] = 206;
        liftHeights[0] = 0;
        lift = hardwareMapG.get(DcMotor.class,"lifter");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void Lift(float delta, boolean liftLimit)
    {
        targetRotations += (int)delta;
        if(targetRotations < 0 && liftLimit){
            targetRotations = 0;
        }
        if(targetRotations > 4200 && liftLimit){
            targetRotations = 4200;
        }
        LiftToTarget();
    }

    public void Lift(int targetPosition){
        targetRotations = liftHeights[targetPosition];
        LiftToTarget();
    }

    private void LiftToTarget()
    {
        lift.setTargetPosition(targetRotations);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(1);
    }

    public  void  Reset(){
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

}
