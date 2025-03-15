// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.goToCoralStation;
//import frc.robot.commandgroups.IntakeInCMD;
//import frc.robot.commandgroups.IntakeOutCMD;
//import frc.robot.commandgroups.closeRobotCMD;
import frc.robot.commands.goToL1;
import frc.robot.commands.goToL2;
import frc.robot.commands.goToL3;
import frc.robot.commands.goToL4;
import frc.robot.commands.goToRest;
//import frc.robot.commands.claw.ClawExitCMD;
//import frc.robot.commands.claw.ClawGoAngledCMD;
//import frc.robot.commands.claw.ClawGoStraight;
//import frc.robot.commands.claw.ClawIntakeCMD;
/*import frc.robot.commands.intake.intakeDownCMD;
import frc.robot.commands.intake.intakeUpCMD;
import frc.robot.commands.intake.intakeWheelCMD;
import frc.robot.commands.intake.intakeWheelReverseCMD;

*/
import frc.robot.subsystems.ARM_SS;
//import frc.robot.subsystems.ClawSS;
//import frc.robot.subsystems.IntakeSS;
/*
import frc.robot.commands.intake.intakeDownCMD;
import frc.robot.commands.intake.intakeUpCMD;
import frc.robot.commands.intake.intakeWheelReverseCMD;
import frc.robot.commands.wrist.WristExitCMD;
import frc.robot.commands.wrist.WristGoToAngledCMD;
import frc.robot.commands.wrist.WristGoToStraightCMD;
import frc.robot.commands.wrist.WristIntakeCMD;
import frc.robot.subsystems.IntakeSS;
import frc.robot.subsystems.WristSS;
 */
//import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import java.lang.reflect.Field;

import swervelib.SwerveInputStream;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

    //subystem declaration
    //private final ClawSS mClawSS = new ClawSS();
    //private final IntakeSS mIntakeSS = new IntakeSS();
    private final ARM_SS mArm_SS = new ARM_SS();
    
    //claw command declaration
    //private final ClawExitCMD mClawExitCMD = new ClawExitCMD(mClawSS);
    //private final ClawGoAngledCMD mClawGoAngledCMD = new ClawGoAngledCMD(mClawSS);
    //private final ClawGoStraight mClawGoStraight = new ClawGoStraight(mClawSS);
    //private final ClawIntakeCMD mClawIntakeCMD = new ClawIntakeCMD(mClawSS);

    //intake command declaration
    //private final intakeDownCMD mIntakeDownCMD = new intakeDownCMD(mIntakeSS);
    //private final intakeUpCMD mIntakeUpCMD = new intakeUpCMD(mIntakeSS);
    //private final intakeWheelCMD mIntakeWheelCMD = new intakeWheelCMD(mIntakeSS);
    //private final intakeWheelReverseCMD mIntakeWheelReverseCMD = new intakeWheelReverseCMD(mIntakeSS);

    //arm position declaration
    private final goToL1 mGoToL1 = new goToL1(mArm_SS);
    private final goToL2 mGoToL2 = new goToL2(mArm_SS);
    private final goToL3 mGoToL3 = new goToL3(mArm_SS);
    private final goToL4 mGoToL4 = new goToL4(mArm_SS);
    private final goToRest mGoToRest = new goToRest(mArm_SS);
    private final goToCoralStation mGoToCoralStation = new goToCoralStation(mArm_SS);

    //command groups declaration
    //private final closeRobotCMD mCloseRobotCMD = new closeRobotCMD(mArm_SS, mClawSS);
    //private final IntakeInCMD mIntakeInCMD = new IntakeInCMD(mIntakeSS, mArm_SS);
    //private final IntakeOutCMD mIntakeOutCMD = new IntakeOutCMD(mIntakeSS, mArm_SS, mClawSS);





// Xbox Controller
    final CommandXboxController driverXbox = new CommandXboxController(0);

// Console box 
    final GenericHID mButtonBoxPT1 = new GenericHID(2);
    final GenericHID mButtonBoxPT2 = new GenericHID(3);

  Trigger btn1 = new Trigger(()->mButtonBoxPT1.getRawButton(1));
  Trigger btn2 = new Trigger(()->mButtonBoxPT1.getRawButton(2));
  Trigger btn3 = new Trigger(()->mButtonBoxPT1.getRawButton(3));
  Trigger btn4 = new Trigger(()->mButtonBoxPT1.getRawButton(4));
  Trigger btn5 = new Trigger(()->mButtonBoxPT1.getRawButton(5));
  Trigger btn6 = new Trigger(()->mButtonBoxPT1.getRawButton(6));
  Trigger btn7 = new Trigger(()->mButtonBoxPT1.getRawButton(7));
  Trigger btn8 = new Trigger(()->mButtonBoxPT1.getRawButton(8));
  Trigger btn9 = new Trigger(()->mButtonBoxPT1.getRawButton(9));
  Trigger btn10 = new Trigger(()->mButtonBoxPT1.getRawButton(10));
  Trigger btn11 = new Trigger(()->mButtonBoxPT1.getRawButton(11));
  Trigger btn12 = new Trigger(()->mButtonBoxPT1.getRawButton(12));
  Trigger btn13 = new Trigger(()->mButtonBoxPT1.getRawButton(13));
  Trigger btn14 = new Trigger(()->mButtonBoxPT1.getRawButton(14));
  Trigger btn15 = new Trigger(()->mButtonBoxPT1.getRawButton(15));
  Trigger btn16 = new Trigger(()->mButtonBoxPT1.getRawButton(16));
  Trigger btn17 = new Trigger(()->mButtonBoxPT1.getRawButton(17));
  Trigger btn18 = new Trigger(()->mButtonBoxPT1.getRawButton(18));
  Trigger btn19 = new Trigger(()->mButtonBoxPT1.getRawButton(19));
  Trigger btn20 = new Trigger(()->mButtonBoxPT1.getRawButton(20));
  Trigger btn21 = new Trigger(()->mButtonBoxPT1.getRawButton(21));
  Trigger btn22 = new Trigger(()->mButtonBoxPT1.getRawButton(22));
  Trigger btn23 = new Trigger(()->mButtonBoxPT1.getRawButton(23));
  Trigger btn24 = new Trigger(()->mButtonBoxPT1.getRawButton(24));

  
  Trigger mbtn1 = new Trigger(()->mButtonBoxPT2.getRawButton(1));
  Trigger mbtn2 = new Trigger(()->mButtonBoxPT2.getRawButton(2));
  Trigger mbtn3 = new Trigger(()->mButtonBoxPT2.getRawButton(3));
  Trigger mbtn4 = new Trigger(()->mButtonBoxPT2.getRawButton(4));
  Trigger mbtn5 = new Trigger(()->mButtonBoxPT2.getRawButton(5));
  Trigger mbtn6 = new Trigger(()->mButtonBoxPT2.getRawButton(6));
  Trigger mbtn7 = new Trigger(()->mButtonBoxPT2.getRawButton(7));
  Trigger mbtn8 = new Trigger(()->mButtonBoxPT2.getRawButton(8));
  Trigger mbtn9 = new Trigger(()->mButtonBoxPT2.getRawButton(9));
  Trigger mbtn10 = new Trigger(()->mButtonBoxPT2.getRawButton(10));
  Trigger mbtn11 = new Trigger(()->mButtonBoxPT2.getRawButton(11));
  Trigger mbtn12 = new Trigger(()->mButtonBoxPT2.getRawButton(12));
  Trigger mbtn13 = new Trigger(()->mButtonBoxPT2.getRawButton(13));
  Trigger mbtn14 = new Trigger(()->mButtonBoxPT2.getRawButton(14));
  Trigger mbtn15 = new Trigger(()->mButtonBoxPT2.getRawButton(15));
  Trigger mbtn16 = new Trigger(()->mButtonBoxPT2.getRawButton(16));
  Trigger mbtn17 = new Trigger(()->mButtonBoxPT2.getRawButton(17));
  Trigger mbtn18 = new Trigger(()->mButtonBoxPT2.getRawButton(18));
  Trigger mbtn19 = new Trigger(()->mButtonBoxPT2.getRawButton(19));
  Trigger mbtn20 = new Trigger(()->mButtonBoxPT2.getRawButton(20));
  Trigger mbtn21 = new Trigger(()->mButtonBoxPT2.getRawButton(21));
  Trigger mbtn22 = new Trigger(()->mButtonBoxPT2.getRawButton(22));
  Trigger mbtn23 = new Trigger(()->mButtonBoxPT2.getRawButton(23));
  Trigger mbtn24 = new Trigger(()->mButtonBoxPT2.getRawButton(24));


  // The robot's subsystems and commands are defined here...
 

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {

   
     
    // Configure the trigger bindings
    DriverStation.silenceJoystickConnectionWarning(true);

    //NamedCommands

   /*  NamedCommands.registerCommand("ClawExitCMD", mClawExitCMD);
    NamedCommands.registerCommand("ClawGoAngled", mClawGoAngledCMD);
    NamedCommands.registerCommand("ClawGoStraight", mClawGoStraight);
    NamedCommands.registerCommand("ClawIntakeCMD", mClawIntakeCMD);
*/
    //NamedCommands.registerCommand("IntakeDownCMD", mIntakeDownCMD);
    //NamedCommands.registerCommand("IntakeUpCMD", mIntakeUpCMD);
    //NamedCommands.registerCommand("IntakeWheelCMD", mIntakeWheelCMD);
    //NamedCommands.registerCommand("IntakeWheelReverseCMD", mIntakeWheelReverseCMD);

    NamedCommands.registerCommand("GoToL1CMD", mGoToL1);
    NamedCommands.registerCommand("GoToL2CMD", mGoToL2);
    NamedCommands.registerCommand("GoToL3CMD", mGoToL3);
    NamedCommands.registerCommand("GoToL4CMD", mGoToL4);
    NamedCommands.registerCommand("GoToRest", mGoToRest);

   /*  NamedCommands.registerCommand("CloseRobot", mCloseRobotCMD);
    NamedCommands.registerCommand("IntakeInCMD", mIntakeInCMD);
    NamedCommands.registerCommand("IntakeOutCMD", mIntakeOutCMD);
*/
  

    configureBindings();


  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {

    // Poteau Positions Bleu


    
    //
   


   
      mbtn8.whileTrue(mGoToRest);
      btn12.whileTrue(mGoToL1);
      btn7.whileTrue(mGoToL2);
      btn3.whileTrue(mGoToL3);
      btn5.whileTrue(mGoToL4);
      mbtn12.whileTrue(mGoToCoralStation);


      //L1-2-3-4
      //ReefPositions

    
    }

  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    //return drivebase.getAutonomousCommand("Straight Auto");
    return null;
    //driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());

  }

  public void setMotorBrake(boolean brake)
  {
    
  }
}