package org.firstinspires.ftc.teamcode.Cheese;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Cheesy Richard", group="Cheese")
//@Disabled
public class CheeseRichard extends AutoSuppliesCheese {
    @Override
    public void runOpMode() {

        //  Establish all hardware
        initForAutonomous();

        //  Wait until start
        waitForStart();

        //R
        pause( 3000 );
        move(3182,0,0.8);
        pause(500);
        move(1500, 0.5,-0.5);
        pause(500);
        move(1500,-0.5, -0.5);
        pause(500);
        move(1500,0.5, -0.5);
        pause(1000);
        turnToS(-90,.5,2);
        pause(500);
        move(1500,0, 0.5);
        pause(500);

        //B
        resetAngle();
        turnToS(90,.5,2);
        pause(500);
        move(4244,0,0.8);
        pause(500);
        move(1500, 0.5,-0.5);
        pause(500);
        move(1500,-0.5, -0.5);
        pause(500);
        move(1500,0.5, -0.5);
        pause(1000);
        move(1500,-0.5, -0.5);
        pause(1000);


        //  Turn all motors off and sleep
        motorFwdLeft.setPower(0);
        motorFwdRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
        sleep(1000);
    }
}
