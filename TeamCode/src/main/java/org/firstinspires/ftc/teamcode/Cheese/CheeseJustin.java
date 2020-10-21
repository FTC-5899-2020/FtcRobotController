package org.firstinspires.ftc.teamcode.Cheese;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Cheesy Justin", group="Cheese")
//@Disabled
public class CheeseJustin extends AutoSuppliesCheese {
    public void charJ(){
        move(2000,0,-.5);
        pause(100);
        turnToS(90,.5,2);
        resetAngle();
        pause(100);
        move(1000,0,.5);
        pause(100);
        turnToS(-90,.5,2);
        resetAngle();
        pause(100);
        move(1000,0,.5);
        pause(100);
        turnToS(-90,.5,2);
        resetAngle();
        pause(100);
        move(2000,0,.5);
        pause(1000);
    }
    public void charK(){
        turnToS(90,.5,2);
        resetAngle();
        pause(100);
        move(500,0,.5);
        pause(1000);
        move(1000,0,-.5);
        pause(1000);
        move(500,0,.5);
        pause(100);
        move(1000,.5,.5);
        pause(100);
        move(1000,-.5,-.5);
        pause(100);
        move(1000,.5,-.5);
        pause(100);
        move(1000,-.5,.5);

    }
    public void driveUntil(){
        while(opModeIsActive()){
            if(getDistanceLeft()>400){
                setPower(0,.5);
            }
            else if(getDistanceRight()<400){
                setPower(0,.5);
            }
            else{
                setPower(0,0);
                resetAngle();
                turnToS(90,.5,2);
            }
        }
    }
    @Override
    public void runOpMode() {

        //  Establish all hardware
        initForAutonomous();

        //  Wait until start
        waitForStart();

        //charJ();
        //charK();
        driveUntil();



        //  Turn all motors off and sleep
        motorFwdLeft.setPower(0);
        motorFwdRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
        sleep(1000);
    }
}
