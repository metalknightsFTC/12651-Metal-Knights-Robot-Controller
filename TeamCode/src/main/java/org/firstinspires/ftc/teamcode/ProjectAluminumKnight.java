package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Enums.Motor;
import org.firstinspires.ftc.teamcode.UtilityClasses.DriveTrainController;
import org.firstinspires.ftc.teamcode.UtilityClasses.IMUController;
import org.firstinspires.ftc.teamcode.UtilityClasses.LiftManager;
import org.firstinspires.ftc.teamcode.UtilityClasses.NavigationManager;
import org.firstinspires.ftc.teamcode.Vectors.Vector2;
import org.firstinspires.ftc.teamcode.Vectors.Vector3;

@TeleOp
public class ProjectAluminumKnight extends LinearOpMode {

    private Servo grabber;
    private Servo verticalR;
    private Servo horizontalR;
    private IMUController imu;
    private DriveTrainController driveTrainController;
    private LiftManager lift;
    private NavigationManager navSystem;
    private static final float slowSpeed = .3f;
    private static final float regularSpeed = .7f;
    private static final float fastSpeed = 1f;
    public static float currentSpeed = .6f;
    private float targetLockHeading;
    private boolean liftLimits = true;
    private boolean hasLocked = false;
    private boolean givenOrder = false;
    private Vector2 target = new Vector2();

    @Override
    public void runOpMode()
    {
        Initialize();
        waitForStart();
        RunLoop();
    }

    public void RunLoop()
    {
        while (opModeIsActive())
        {
            SetCameraAngle(0,.15);
            LiftRecovery();

            SpeedAndDrive();
            ManageLiftLevel();
            //Contact();
            //region Grabber Code
            if(gamepad1.left_bumper){
                grabber.setPosition(0.16f);
            }else{
                grabber.setPosition(.33f);
            }
            //endregion

            imu.GetAngle();
            telemetry.addData("Angle : ", imu.heading);
            telemetry.update();
        }
    }

    public void SpeedAndDrive()
    {
        if(gamepad1.right_bumper)
        {
            currentSpeed = slowSpeed;
        }else if(gamepad1.left_stick_button)
        {
            currentSpeed = fastSpeed;
        }else
        {
            currentSpeed = regularSpeed;
        }
        if(!gamepad1.back)
        {
            target = Vector2.zero();
            givenOrder = false;
            navSystem.ResetNavigationSystem();
            if(driveTrainController.RSX >= 0.001 || driveTrainController.RSX <= -0.001 ||
                    !(driveTrainController.LSX > 0.02f || driveTrainController.LSX < -0.02f) && !gamepad1.right_stick_button)
            {
                imu.ResetAngle();
                driveTrainController.UpdateDriveTrain(currentSpeed, StrafeCorrection());
            }
            else
            {
                SoulsLikeTargetLock();
                driveTrainController.UpdateDriveTrain(currentSpeed,0);
            }
        }else{
            if(!givenOrder) {
                givenOrder = true;
            }
            if(gamepad1.dpad_up)
            {
                target = new Vector2(0,-2);
            }
            if(gamepad1.dpad_down)
            {
                target = new Vector2(0,2);
            }
            if(gamepad1.dpad_left)
            {
                target = new Vector2(2,0);
            }
            if(gamepad1.dpad_right)
            {
                target = new Vector2(-2,0);
            }
            navSystem.MoveTO(0,0,0);
            telemetry.addData("Debug: ", navSystem.totalMovementZ);
            float xPow = (float) navSystem.totalMovementX - target.x;
            float zPow = (float) navSystem.totalMovementZ - target.z;
            xPow = Range(xPow,-1,1) * currentSpeed;
            zPow = Range(zPow,-1,1) * currentSpeed;
            if(navSystem.totalMovementZ < target.z - .2 || navSystem.totalMovementZ > target.z + .2
                    || navSystem.totalMovementX < target.x - .2 || navSystem.totalMovementX > target.x + .2)
            {
                driveTrainController.UpdateDriveTrain(new Vector3(xPow, StrafeCorrection() * currentSpeed, zPow));
            }else
            {
                driveTrainController.UpdateDriveTrain(Vector3.zero());
            }
        }
    }

    public void ManageLiftLevel()
    {
        //region lifter buttons
        //538
        if(liftLimits)
        {
            if (gamepad1.dpad_right)
            {
                lift.Lift(4);
            }
            if (gamepad1.dpad_left)
            {
                lift.Lift(2);
            }
            if (gamepad1.dpad_up)
            {
                lift.Lift(3);
            }
            if (gamepad1.dpad_down)
            {
                lift.Lift(1);
            }
            if (gamepad1.a)
            {
                lift.Lift(5);
            }
            if (gamepad1.x)
            {
                lift.Lift(6);
            }
            if (gamepad1.y)
            {
                lift.Lift(7);
            }
            if (gamepad1.b)
            {
                lift.Lift(0);
            }
        }
        //endregion
        //region lifter code
        lift.Lift((gamepad1.right_trigger - gamepad1.left_trigger) * 20f, liftLimits);
        //endregion
    }

    //region Initialization Code
    public void Initialize(){
        Blinker expansion_Hub_1 = hardwareMap.get(Blinker.class, "Control Hub");
        Blinker expansion_Hub_2 = hardwareMap.get(Blinker.class, "Expansion Hub 1");
        imu = new IMUController(hardwareMap);
        lift = new LiftManager(hardwareMap);
        horizontalR = hardwareMap.get(Servo.class, "alignment");
        verticalR = hardwareMap.get(Servo.class, "pivot");
        grabber = hardwareMap.get(Servo.class,"grabber");
        driveTrainController = new DriveTrainController(gamepad1,hardwareMap);
        driveTrainController.InvertMotorDirection(Motor.backLeft);
        driveTrainController.InvertMotorDirection(Motor.frontLeft);
        imu.ResetAngle();
        navSystem = new NavigationManager(hardwareMap, imu, driveTrainController);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }
    //endregion
    //region Drift Correction
    public float StrafeCorrection()
    {
        float turnMod = (float) imu.AngleDeviation(0) / 20;
        turnMod = Range(turnMod,-1,1);
        return -turnMod;
    }
    //endregion
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

    //region Target Lock
    public void SoulsLikeTargetLock()
    {
        if(!gamepad1.right_stick_button)
        {
            hasLocked = false;
        }
        if(!hasLocked && gamepad1.right_stick_button) {
            targetLockHeading = imu.GetAngle();
            hasLocked = true;
        }
        if(hasLocked)
        {
            navSystem.SnapToHeading(targetLockHeading,.4f);
        }else
        {
            targetLockHeading = 0;
        }
    }
    //endregion
    //region Lift Recovery
    public void LiftRecovery()
    {
        telemetry.addData("Recover Lift: ","Hold back to unlock while holding back" +
                " press b to re-lock");
        liftLimits = !gamepad1.back;
        if(gamepad1.back && gamepad1.b)
        {
            telemetry.addData("Lift Recovery: ", "Re-locked");
            lift.Reset();
        }
    }
    //endregion

    private void SetCameraAngle(double x, double z)
    {
        verticalR.setPosition(z);
        horizontalR.setPosition(x);
    }

}