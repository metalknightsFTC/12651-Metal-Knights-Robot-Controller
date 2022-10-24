package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Vectors.*;

import org.firstinspires.ftc.teamcode.Enums.Motor;

public class DriveTrainCode {

    public double LSX;
    public double LSY;
    public double RSX;
    private  DcMotor frontRight;
    private  DcMotor frontLeft;

    private  DcMotor backRight;
    private  DcMotor backLeft;
    //local reference to gamepad 1 passed by ref to constructor
    private Gamepad lGpad;
    HardwareMap lHardwareMap;
    //double speed = 1;
    public float frTicks;
    public float brTicks;
    public float flTicks;
    public float blTicks;

    public  DriveTrainCode(Gamepad gamepad , HardwareMap hardwareMap)
    {
        InitializeGamepad(gamepad);
        InitializeHardware(hardwareMap);
    }
    public void  UpdateDriveTrain(float speed, float strafeCorrection)
    {
        UpdateInput();
        UpdateMecanum(speed, strafeCorrection);
    }

    public void  UpdateDriveTrain(Vector3 direction)
    {
        SimulateStick(direction.x,direction.z,direction.y);
    }


    private  void UpdateMecanum(float speed, float strafeCorrection)
    {

        frontLeft.setPower(((LSY)-(RSX+strafeCorrection)-(LSX)) * speed);
        backLeft.setPower(((LSY)-(RSX+strafeCorrection)+(LSX)) * speed);

        frontRight.setPower(((LSY)+(RSX+strafeCorrection)+(LSX)) * speed);
        backRight.setPower(((LSY)+(RSX+strafeCorrection)-(LSX)) * speed);
    }

    public  void InvertMotorDirection(Motor selectedMotor)
    {
        if (selectedMotor == Motor.frontRight){
            frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        }else if (selectedMotor == Motor.frontLeft){
            frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        }else if(selectedMotor == Motor.backRight){
            backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        } else if (selectedMotor == Motor.backLeft) {
            backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        }
    }

    private void  SimulateStick(float x, float y, float t)
    {
        frontLeft.setPower((y)+(t)+(x));
        backLeft.setPower(((y)+(t)-(x)));
        frontRight.setPower(((y)-(t)-(x)));
        backRight.setPower(((y)-(t)+(x)));

        frTicks = frontRight.getCurrentPosition();
        brTicks = backRight.getCurrentPosition();

        flTicks = frontLeft.getCurrentPosition();
        blTicks = backLeft.getCurrentPosition();
    }

    private void UpdateInput()
    {
        LSX = Math.pow(lGpad.left_stick_x, 3);
        LSY = Math.pow(lGpad.left_stick_y, 3);
        RSX = lGpad.right_stick_x;
    }

    private  void  InitializeHardware(HardwareMap hardwareMap)
    {
        lHardwareMap = hardwareMap;
        frontRight = lHardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = lHardwareMap.get(DcMotor.class, "frontLeft");

        backRight = lHardwareMap.get(DcMotor.class, "backRight");
        backLeft = lHardwareMap.get(DcMotor.class, "backLeft");
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private  void InitializeGamepad(Gamepad gpad)
    {
        lGpad = gpad;
    }

}
