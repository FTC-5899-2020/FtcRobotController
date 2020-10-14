package org.firstinspires.ftc.teamcode.TestPrograms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="FlyWheelTest", group="Test Group")
//@Disabled
public class FlyWheelTest extends LinearOpMode{

    //All hardware
    protected DcMotor motorRight = null;
    protected DcMotor motorLeft = null;


    //Variables
    double powerLim = 0;

    @Override
    public void runOpMode() {
        motorRight = hardwareMap.get(DcMotor.class, "motorRight");
        motorLeft = hardwareMap.get(DcMotor.class, "motorLeft");
        motorRight.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if(this.gamepad1.dpad_up){
                powerLim+=.001;
            }
            else if(this.gamepad1.dpad_down){
                powerLim-=.001;
            }
            else if(this.gamepad1.a){
                powerLim = .5;
            }
            else if(this.gamepad1.x){
                powerLim = .75;
            }
            else if(this.gamepad1.y){
                powerLim = 1;
            }
            else if(this.gamepad1.b){
                powerLim = 0;
            }
            //telemetry
            telemetry.addData("Power",powerLim);
            telemetry.update();
            //update motor power
            motorRight.setPower(powerLim);
            motorLeft.setPower(powerLim);
        }
    }
}
