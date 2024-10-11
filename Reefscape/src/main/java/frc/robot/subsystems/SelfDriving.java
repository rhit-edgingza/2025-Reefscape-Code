/*
Custom code that will help the robot know how to self drive

Goals 
    1) animate driving to help assistdriver
    2) driving to game piece 
    3) use lane logic to navigate know absticals on the feild


System is fully functional if PIDs, lines, and target are given correctly 

 */

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.units.Current;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.Swerve.MoveToNoteByPose;
import frc.robot.commands.Swerve.TeleopSwerve;
import frc.robot.subsystems.SwerveBase;

public class SelfDriving extends  SubsystemBase{
   
public static double TargetPoseX;
public static double TargetPoseY;
public static double TargetPoseRotation;

public static double StartingTargetPoseX;
public static double StartingTargetPoseY;
public static double StartingTargetPoseRotation;

public static double PoseDifferenceX;
public static double PoseDifferenceY;
public static double PoseDifferenceRotation;
public static double PoseDifferenceRotationRaw;

public static double XSpeed;
public static double YSpeed;
public static double RotationSpeed;

public static double XSpeedFinal;
public static double YSpeedFinal;
public static double RotationSpeedFinal;

public static boolean CloseEnough;
public static double HowFar;
public static double HowStraght;
public static boolean TranslationClose;
public static boolean RotationClose; 


public static boolean AutoSetUpMoveLeft;
public static boolean AutoSetUpMoveRight;
public static boolean AutoSetUpMoveForward;
public static boolean AutoSetUpMoveBackward;
public static boolean AutoSetUpRotateCounterClockWise;
public static boolean AutoSetUpRotateCloskWise;
public static boolean AutoSetUpCloseEnough;



public static double LaneLogicPoseX;
public static double LaneLogicPoseY;
public static double LaneLogicPoseRotation;
public static boolean LaneLogicFinalDestination = false;
// public static boolean AutoSetUpRotateCounterClockWise;
// public static boolean AutoSetUpRotateCloskWise;
// public static boolean AutoSetUpCloseEnough;


//Value requested will not go over this value
public static double MaxTranslationSpeed = 1.0;//3.5
public static double MaxRotationSpeed = Math.PI;


//Make the value reqest from the drive ramp the power to get to target
//WARNING this could mess with deceleration if PID decelerates faster then is value
//to fix issue lower this but would prefer for it to be high for smoother motion
//COmment this out when first tuning PIDS
/*Slew Limiter */
static SlewRateLimiter MaxTranslationAccelerationX = new SlewRateLimiter(.55);
static SlewRateLimiter MaxTranslationAccelerationY = new SlewRateLimiter(.55);
static SlewRateLimiter MaxRotationAcceleration = new SlewRateLimiter(2*Math.PI);

// PIDs to Contol Driving to Note
public static PIDController NoteTranslation = new PIDController(0.285, 0, 0.0);
public static PIDController NoteRotation = new PIDController(0.325, 0, 0);

// PIDs to Control Driving to Pose  
public static PIDController MoveToPoseTranslation = new PIDController(0.285, 0.0, 0.0);
public static PIDController MoveToPoseRotation = new PIDController(0.00895, 0, 0);




    //This method will take the current Pose and compare it to the Pose of starting auto and will give variables to the LED 
    //Needs to be Test 
    public void AutoSetUp() {

     
       //If you want to change the order of auto set up change the order in which it checks for error
        if(Math.abs(PoseDifferenceY) > 0.015){
                if(PoseDifferenceY > 0){//Positive Moves to Right
                    AutoSetUpMoveRight = true;
                }
                else{//Negative Moves to Left
                    AutoSetUpMoveLeft = true;
                } 
        }
        else if(Math.abs(PoseDifferenceRotationRaw) > 1){
                if(PoseDifferenceRotationRaw > 0){//Need to double check but i think Positve should rotate counterclock wise
                    AutoSetUpRotateCounterClockWise = true;
                }
                else{//Need to double check but i think Negative should rotate clock wise
                    AutoSetUpRotateCounterClockWise = true;
                }
        }
        else if(Math.abs(PoseDifferenceX) > 0.015){
                if(PoseDifferenceX > 0){//Positive Moves to Backward
                    AutoSetUpMoveBackward = true;
                }
                else{//Negative Moves to 
                    AutoSetUpMoveForward = true;
                }
        }
        else{//Robot is within set distances and is ready for auto
                    AutoSetUpCloseEnough = true;
        }

    }

    //This method will be used to help drive the robot to the game pieces
    public static void DriveCalculationNote() {

    XSpeed = -SelfDriving.NoteTranslation.calculate(NoteDetection.MainXDistance, 0.3);
    RotationSpeed = SelfDriving.NoteRotation.calculate(NoteDetection.MainYDistance, 0.0);

 // Sets a max out put for variable
    XSpeed =  Math.abs(XSpeed) < MaxTranslationSpeed ?  XSpeed : MaxTranslationSpeed;
    YSpeed =  Math.abs(YSpeed) < MaxTranslationSpeed ?  YSpeed : MaxTranslationSpeed;
    RotationSpeed =  Math.abs(RotationSpeed) < MaxRotationSpeed ?  RotationSpeed : MaxRotationSpeed; 

    //Apply the accelertion limiter
    XSpeedFinal =  MaxTranslationAccelerationX.calculate(XSpeed);
    RotationSpeedFinal = MaxRotationAcceleration.calculate(RotationSpeed);

}

//The following method sets the target for the self driving systems 
//Example of calling with a button: btn_self_driving_shoot.whileTrue(new RunCommand(() -> m_selfDriving.setTargetPose(1)));
public void setTargetPose(int set) {
if (set == 0){}

//Shotting Infront of Speacker
if (set == 1){ 
    //Pose and Angle For Red
    if (Constants.isRed == true){
    TargetPoseX = 15.0;
    TargetPoseY = 5.55;
    TargetPoseRotation = 0;
        }
    //Pose and Angle For Blue
    if (Constants.isRed == false){
    TargetPoseX = 1.38;
    TargetPoseY = 5.55;
    TargetPoseRotation = 0;
        }
    }
//Amp Pose 
if (set == 2){ 
    //Pose and Angle For Red
    if (Constants.isRed == true){
    TargetPoseX = 14.7;
    TargetPoseY = 7.6;
    TargetPoseRotation = 90;
        }
    //Pose and Angle For Blue
    if (Constants.isRed == false){
    TargetPoseX = 1.80;
    TargetPoseY = 7.60;
    TargetPoseRotation = 90;
        }

    }

    //Note Pose 
        if (set == 3){

            TargetPoseX = NoteDetection.MainNoteXRobot + SwerveBase.currentPoseX;
            TargetPoseY = NoteDetection.MainNoteYRobot + SwerveBase.currentPoseY;
            // TargetPoseRotation = RobotContainer.angleController.calculate(m_swerveBase.getPose().getRotation().getRadians(), MoveToNoteByPose.NOTE_POSE.getRotation().getRadians());
        }
//Starting position for auto A
        if (set == 4){ 
    //Pose and Angle For Red
    if (Constants.isRed == false){
    TargetPoseX = 14.7;
    TargetPoseY = 7.6;
    TargetPoseRotation = 90;
        }
    //Pose and Angle For Blue
    if (Constants.isRed == true){
    TargetPoseX = 1.80;
    TargetPoseY = 7.60;
    TargetPoseRotation = 90;
        }
    }
    //Call when Lane Logic is active to drive to destination
        if (set == 10){

            TargetPoseX = LaneLogicPoseX;
            TargetPoseY = LaneLogicPoseY;
            TargetPoseRotation = LaneLogicPoseRotation;
        
        }
    }

//The following methed need to be call when selfdriving is used
public static void DriveCalculationPose() {
  


    //Because we are driving in feild centric for this command the number will change signs depending on allaince
  
   if (Constants.isRed == true){
        PoseDifferenceX = TargetPoseX - SwerveBase.currentPoseX;
        PoseDifferenceY = TargetPoseY - SwerveBase.currentPoseY;// Need to test might not be right
        PoseDifferenceRotationRaw = SwerveBase.currentPoseRotation -TargetPoseRotation; // Need to test might not be right   
    }
        if (Constants.isRed == false){
        PoseDifferenceX = SwerveBase.currentPoseX - TargetPoseX;
        PoseDifferenceY = SwerveBase.currentPoseY - TargetPoseY;// Need to test might not be right
        PoseDifferenceRotationRaw = SwerveBase.currentPoseRotation -TargetPoseRotation;// Need to test might not be right
    }
//Roation should be the same regardless of alliance
//The following will optimize the rotation to go left or right
        if (PoseDifferenceRotationRaw > 180){
        PoseDifferenceRotation = PoseDifferenceRotationRaw - 360;
    }
    //  if (PoseDifferenceRotationRaw < -180){
    //     PoseDifferenceRotation = PoseDifferenceRotationRaw + 360;
    // }
        else {
        PoseDifferenceRotation =  PoseDifferenceRotationRaw;
    }

  
    //Apllies PID to move to Target Pose
    XSpeed =  MoveToPoseTranslation.calculate(PoseDifferenceX, 0.0);
    YSpeed =  MoveToPoseTranslation.calculate(PoseDifferenceY, 0.0);
    RotationSpeed = MoveToPoseRotation.calculate(PoseDifferenceRotation, 0.0);
   
    //Sets a max out put for variable
    XSpeed =  Math.abs(XSpeed) < MaxTranslationSpeed ?  XSpeed : MaxTranslationSpeed;
    YSpeed =  Math.abs(YSpeed) < MaxTranslationSpeed ?  YSpeed : MaxTranslationSpeed;
    RotationSpeed =  Math.abs(RotationSpeed) < MaxRotationSpeed ?  RotationSpeed : MaxRotationSpeed; 

    //Apply the accelertion limiter 
    XSpeedFinal = MaxTranslationAccelerationX.calculate(XSpeed);
    YSpeedFinal = MaxTranslationAccelerationY.calculate(YSpeed);
    RotationSpeedFinal = MaxRotationAcceleration.calculate(RotationSpeed);

    // When CloseEnough == Fasle MoveToPose will end
    //This is where you will specify how close it must be for what you are doing to work
    if( (Math.abs(PoseDifferenceRotation)) < 5.0 && (Math.abs(PoseDifferenceY)) < 0.15 && (Math.abs(PoseDifferenceX)) < 0.15){
        CloseEnough = true;
    }
    else{
        CloseEnough = false;
    }

}


    //The following method is the lane logic this will seperate the feild into parts and define a place to drive based on where it is
    //Will have to be called in command or button to activate

    /*Writing Reminders
     * 1. Every quadrant should be constraned on all four sides or more
     * 2. Desired location should not be within the quadrent unless it is the final destination
     * 3. Robot will only drive in a staight line so make quadrants accordingly
     * 4. && = and
     * 5. & = or
     * 6. Make all quadrants and name them before starting
     * 7. Every part of the feild need a quadrent
     * 8. To make code shorter define line then quadrent based on thoose lines
     * 9. Lane to do need to change based on the allaince
     */

        //The following method will be used to drive the bot towards the drivers station
        public static void LaneLogicIn() {
            //The following is the Red logic
            if(Constants.isRed == true){
               
                //Quadrant 2
                if(5.598 < SwerveBase.currentPoseX && SwerveBase.currentPoseX < 10.914  && 5.408 < SwerveBase.currentPoseY  && SwerveBase.currentPoseY < 8.237 ){
                    LaneLogicPoseX = 11.914 ;
                    LaneLogicPoseY = 7.050;
                    LaneLogicPoseRotation = 110;
                    System.out.println("Quadrant 2 ");
                }

                //Quadrant 3
                if(10.914 < SwerveBase.currentPoseX && SwerveBase.currentPoseX < 16.509  && 6.075 < SwerveBase.currentPoseY  && SwerveBase.currentPoseY < 8.237 ){
                    LaneLogicPoseX = 15.81;
                    LaneLogicPoseY = 6.804;
                    LaneLogicPoseRotation = 110;
                    System.out.println("Quadrant 3");

                 }


                 //Quadrant 7
                if(9.769 > SwerveBase.currentPoseX && SwerveBase.currentPoseX > 6.743  && 5.408 > SwerveBase.currentPoseY  && SwerveBase.currentPoseY > 2.423){
                    LaneLogicPoseX = 12.00;
                    LaneLogicPoseY = 4.78;
                    LaneLogicPoseRotation = 180;
                    System.out.println("Quadrant 7");
                 }

                 //Quadrant 8
                 if(9.769 < SwerveBase.currentPoseX &&   -1.7140 *(SwerveBase.currentPoseX - 10.914) + 5.408 > SwerveBase.currentPoseY  &&  SwerveBase.currentPoseY > 0.5682 *(SwerveBase.currentPoseX - 15.618) + 5.046 && SwerveBase.currentPoseY < 5.408 && SwerveBase.currentPoseY > 2.423){
                    LaneLogicPoseX = 12.40;
                    LaneLogicPoseY = 5.24;
                    LaneLogicPoseRotation = 180;
                    System.out.println("Quadrant 8");

                 }
                 //Quadrant 9
                 if(10.914 < SwerveBase.currentPoseX && SwerveBase.currentPoseX < 13.469  &&  -1.7140 *(SwerveBase.currentPoseX - 10.914) + 5.408 < SwerveBase.currentPoseY  && SwerveBase.currentPoseY < -.8853 *(SwerveBase.currentPoseX - 10.914) + 6.075 &&  SwerveBase.currentPoseY > 0.5682 *(SwerveBase.currentPoseX - 15.618) + 5.046){
                // if(10.914 < SwerveBase.currentPoseX && SwerveBase.currentPoseX < 13.469  &&  -1.7140 *(SwerveBase.currentPoseX - 10.914) + 5.408 < SwerveBase.currentPoseY  && SwerveBase.currentPoseY < -.8853 *(SwerveBase.currentPoseX - 10.914) + 6.075 &&  SwerveBase.currentPoseY > 0.5682 *(SwerveBase.currentPoseX - 15.618) + 5.046){
                    LaneLogicPoseX = 12.60;
                    LaneLogicPoseY = 5.75;
                    LaneLogicPoseRotation = 180;
                    System.out.println("Quadrant 9");
                 }
                 //Quadrant 10
                if(10.914 < SwerveBase.currentPoseX && SwerveBase.currentPoseX < 16.509  && -.8853 *(SwerveBase.currentPoseX - 10.914) + 6.075 < SwerveBase.currentPoseY  && SwerveBase.currentPoseY > .5682 *(SwerveBase.currentPoseX - 15.618) + 5.046 && SwerveBase.currentPoseY < 6.075){  
                    LaneLogicPoseX = 15.129;
                    LaneLogicPoseY = 5.536;
                    LaneLogicPoseRotation = 180;
                    System.out.println("Quadrant 10 ");

                 }

                 //Quadrant 12
                if(5.858 < SwerveBase.currentPoseX && SwerveBase.currentPoseX < 10.705  && 0 < SwerveBase.currentPoseY  && SwerveBase.currentPoseY < 2.423 ){
                    LaneLogicPoseX = 12.031;
                    LaneLogicPoseY = 2.50;
                    LaneLogicPoseRotation = 180;
                    System.out.println("Quadrant 12 ");

                 }
                 //Quadrant 13
                else if(10.705 < SwerveBase.currentPoseX && SwerveBase.currentPoseX < 15.50  && 0 < SwerveBase.currentPoseY  && SwerveBase.currentPoseY < .5682 *(SwerveBase.currentPoseX - 15.618) + 5.046 ){
                    LaneLogicPoseX = 15.755;
                    LaneLogicPoseY = 4.237;
                    LaneLogicPoseRotation = 240;
                    System.out.println("Quadrant 13"); 
                } 
                
            }
         }
               
        // The following method will be used to drive the bot away from driver station
        public static void LaneLogicOut() {
            //The following is the Blue logic

                
                // if(Constants.isRed == true){}

                // //Quadrant 2
                // if(5.598 < SwerveBase.currentPoseX && SwerveBase.currentPoseX < 10.914  && 5.408 < SwerveBase.currentPoseY  && SwerveBase.currentPoseY < 8.237 ){
                //     LaneLogicPoseX = 15.81;
                //     LaneLogicPoseY = 6.804;
                //     LaneLogicPoseRotation = 180;
                //     System.out.println("Quadrant 2 ");
                // }

                //Quadrant 3
                 if(10.914 < SwerveBase.currentPoseX && SwerveBase.currentPoseX < 16.509  && 6.075 < SwerveBase.currentPoseY  && SwerveBase.currentPoseY < 8.237 ){
                    LaneLogicPoseX = 9.50;
                    LaneLogicPoseY = 6.50;
                    LaneLogicPoseRotation = 180;
                    System.out.println("Quadrant 3");

                 }


                //  //Quadrant 7
                //   if(9.769 > SwerveBase.currentPoseX && SwerveBase.currentPoseX > 6.743  && 5.408 > SwerveBase.currentPoseY  && SwerveBase.currentPoseY > 2.423){
                //     LaneLogicPoseX = 15.129;
                //     LaneLogicPoseY = 5.536;
                //     LaneLogicPoseRotation = 0;
                //     System.out.println("Quadrant 7");
                // }
                 //Quadrant 8
                  if(9.769 < SwerveBase.currentPoseX &&   -1.7140 *(SwerveBase.currentPoseX - 10.914) + 5.408 > SwerveBase.currentPoseY  &&  SwerveBase.currentPoseY > 0.5682 *(SwerveBase.currentPoseX - 15.618) + 5.046 && SwerveBase.currentPoseY < 5.408 && SwerveBase.currentPoseY > 2.423){
                    LaneLogicPoseX = 8.07;
                    LaneLogicPoseY = 4.08;
                    LaneLogicPoseRotation = 180;
                    System.out.println("Quadrant 8");
                 }
                 //Quadrant 9
                  if(10.914 < SwerveBase.currentPoseX && SwerveBase.currentPoseX < 13.469  &&  -1.7140 *(SwerveBase.currentPoseX - 10.914) + 5.408 < SwerveBase.currentPoseY  && SwerveBase.currentPoseY < -.8853 *(SwerveBase.currentPoseX - 10.914) + 6.075 &&  SwerveBase.currentPoseY > 0.5682 *(SwerveBase.currentPoseX - 15.618) + 5.046){
                    LaneLogicPoseX = 10.04;
                    LaneLogicPoseY = 3.63;
                    LaneLogicPoseRotation = 180;
                    System.out.println("Quadrant 9");
                 }
                 //Quadrant 10
                  if(10.914 < SwerveBase.currentPoseX && SwerveBase.currentPoseX < 16.509  && -.8853 *(SwerveBase.currentPoseX - 10.914) + 6.075 < SwerveBase.currentPoseY  && SwerveBase.currentPoseY > .5682 *(SwerveBase.currentPoseX - 15.618) + 5.046 && SwerveBase.currentPoseY < 6.075){
                    LaneLogicPoseX = 11.68; 
                    LaneLogicPoseY = 4.41; 
                    LaneLogicPoseRotation = 180;
                    System.out.println("Quadrant 10 ");

                 }

                //  //Quadrant 12
                // else if(5.858 < SwerveBase.currentPoseX && SwerveBase.currentPoseX < 10.705  && 0 < SwerveBase.currentPoseY  && SwerveBase.currentPoseY < 2.423 ){
                //     LaneLogicPoseX = 15.755;
                //     LaneLogicPoseY = 4.237;
                //     LaneLogicPoseRotation = 0;
                //     System.out.println("Quadrant 12 ");

                //  }
                 //Quadrant 13
                if(10.705 < SwerveBase.currentPoseX && SwerveBase.currentPoseX < 15.50  && 0 < SwerveBase.currentPoseY  && SwerveBase.currentPoseY < .5682 *(SwerveBase.currentPoseX - 15.618) + 5.046 ){
                    LaneLogicPoseX = 9.50;
                    LaneLogicPoseY = 1.75;
                    LaneLogicPoseRotation =  180;
                    System.out.println("Quadrant 13");
                }
                // if(SwerveBase.currentPoseX > 0 && SwerveBase.currentPoseY > 0){
                }
             
           

   @Override
  public void periodic() {

    // System.out.println("Slew" + MaxTranslationAcceleration.calculate(XSpeed)); 


            }
        }

