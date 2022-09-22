package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Enums.SelectedDrive;

@TeleOp
public class ProjectAluminumKnight extends LinearOpMode {

    final String lift = "lifter";
    final String grabber = "grabber";
    SubSystemControl subSystemControl;
    private boolean automatics = false;
    private int targetRotations = 0;

    @Override
    public void runOpMode(){
        /*intake = hardwareMap.get(DcMotor.class, "intake");
        duck = hardwareMap.get(DcMotor.class, "duck");
        lift = hardwareMap.get(DcMotor.class, "lifter");
        grabber = hardwareMap.get(Servo.class, "grabber");*/
        String[] motors = new String[1];
        String[] servos= new String[1];

        motors[0] = lift;
        servos[0] = grabber;

        DriveTrainCode driveTrainCode = new DriveTrainCode(gamepad1,hardwareMap);
        subSystemControl = new SubSystemControl(hardwareMap,motors, servos);

        //driveTrainCode.InvertMotorDirection(Motor.frontRight);
        //driveTrainCode.InvertMotorDirection(Motor.backLeft);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()){
            driveTrainCode.UpdateDriveTrain(SelectedDrive.mecanum);
            subSystemControl.ManipulateMotor(lift).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            subSystemControl.ManipulateMotor(lift).setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            subSystemControl.ManipulateMotor(lift).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            if(gamepad1.back && !automatics){
                automatics = true;
            }else if(gamepad1.back){
                automatics = false;
                targetRotations = 0;
            }
            if(gamepad1.left_bumper){
                subSystemControl.ManipulateServo(grabber).setPosition(1);
            }else{
                subSystemControl.ManipulateServo(grabber).setPosition(0);
            }

            if(gamepad1.a){
                targetRotations = 58;
            }
            if(gamepad1.x){
                targetRotations = 144;
            }
            if(gamepad1.y){
                targetRotations = 288;
            }
            if(gamepad1.b){
                targetRotations = 0;
            }

            telemetry.addData("AP = ", automatics);

            if(automatics){
                double liftDelta = (targetRotations - (subSystemControl.ManipulateMotor(lift).getCurrentPosition()) * .2);
                subSystemControl.ManipulateMotor(lift).setPower(liftDelta);

            }else{
                if(gamepad1.dpad_up)
                    subSystemControl.ManipulateMotor(lift).setPower(0.5);
                else if(gamepad1.dpad_down)
                    subSystemControl.ManipulateMotor(lift).setPower(-0.5);
                else
                    subSystemControl.ManipulateMotor(lift).setPower(0);
            }

            telemetry.addData("Lift Target: ", ((double)targetRotations));
            telemetry.addData("Current Lift Position: ", subSystemControl.ManipulateMotor(lift).getCurrentPosition());
            telemetry.update();

        }

    }
}
