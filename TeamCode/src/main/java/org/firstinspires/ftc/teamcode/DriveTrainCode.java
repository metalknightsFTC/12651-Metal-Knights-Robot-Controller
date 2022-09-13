package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Enums.Motor;
import org.firstinspires.ftc.teamcode.Enums.SelectedDrive;


public class DriveTrainCode {

    private double speedMultiplier;
    private double LSX;
    private double LSY;
    private double RSX;
    private  DcMotor frontRight;
    private  DcMotor frontLeft;
    private  DcMotor backRight;
    private  DcMotor backLeft;
    //local reference to gamepad 1 passed by ref to constructor
    private Gamepad lGpad;
    private HardwareMap lHardwareMap;

    public  DriveTrainCode(Gamepad Gpad , HardwareMap hardwareMap){
        InitializeGamepad(Gpad);
        InitializeHardware(hardwareMap);
    }

    public void  UpdateDriveTrain(SelectedDrive driveType){
        UpdateInput();
        if (driveType == SelectedDrive.mecanum){
            UpdateMecanum();
        }else if (driveType == SelectedDrive.tank){
            UpdateTank();
        }else if (driveType == SelectedDrive.autonomous){
            SimulateStick(0,0,0);
        }
    }

    public void  UpdateDriveTrain(SelectedDrive driveType,float x,float y,float t) {
        UpdateInput();
        if (driveType == SelectedDrive.mecanum) {
            UpdateMecanum();
        } else if (driveType == SelectedDrive.tank) {
            UpdateTank();
        } else if (driveType == SelectedDrive.autonomous) {
            SimulateStick(x,y,t);
        }
    }


    private  void UpdateMecanum(){
        frontLeft.setPower((LSY)-(RSX)-(LSX));
        backLeft.setPower((LSY)-(RSX)+(LSX));

        frontRight.setPower((LSY)+(RSX)+(LSX));
        backRight.setPower((LSY)+(RSX)-(LSX));

    }

    public  void InvertMotorDirection(Motor selectedMotor){
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

    private void  SimulateStick(float x, float y, float t){

        frontLeft.setPower((y)-(t)-(x));
        backLeft.setPower((y)-(t)+(x));
        frontRight.setPower((y)+(t)+(x));
        backRight.setPower((y)+(t)-(x));

    }

    private  void  UpdateTank(){
        frontLeft.setPower(LSY-RSX);
        backLeft.setPower((LSY)-(RSX));
        frontRight.setPower((LSY)+(RSX));
        backRight.setPower((LSY)+(RSX));
    }

    private void UpdateInput(){
        LSX = lGpad.left_stick_x;
        LSY = lGpad.left_stick_y;
        RSX = lGpad.right_stick_x;
    }

    private  void  InitializeHardware(HardwareMap hardwareMap)
    {
        lHardwareMap = hardwareMap;
        frontRight = lHardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = lHardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = lHardwareMap.get(DcMotor.class, "backRight");
        backRight = lHardwareMap.get(DcMotor.class, "backLeft");
/*
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
*/
    }

    private  void InitializeGamepad(Gamepad Gpad)
    {
        lGpad = Gpad;
    }

}
