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

import frc.robot.commandgroups.IntakeInCMD;
import frc.robot.commandgroups.IntakeOutCMD;
import frc.robot.commandgroups.closeRobotCMD;
import frc.robot.commands.goToL1;
import frc.robot.commands.goToL2;
import frc.robot.commands.goToL3;
import frc.robot.commands.goToL4;
import frc.robot.commands.goToRest;
import frc.robot.commands.claw.ClawExitCMD;
import frc.robot.commands.claw.ClawGoAngledCMD;
import frc.robot.commands.claw.ClawGoStraight;
import frc.robot.commands.claw.ClawIntakeCMD;
import frc.robot.commands.intake.intakeDownCMD;
import frc.robot.commands.intake.intakeUpCMD;
import frc.robot.commands.intake.intakeWheelCMD;
import frc.robot.commands.intake.intakeWheelReverseCMD;
import frc.robot.subsystems.ARM_SS;
import frc.robot.subsystems.ClawSS;
import frc.robot.subsystems.IntakeSS;
 

import frc.robot.commands.intake.intakeDownCMD;
import frc.robot.commands.intake.intakeUpCMD;
import frc.robot.commands.intake.intakeWheelReverseCMD;
//import frc.robot.commands.wrist.WristExitCMD;
//import frc.robot.commands.wrist.WristGoToAngledCMD;
//import frc.robot.commands.wrist.WristGoToStraightCMD;
//import frc.robot.commands.wrist.WristIntakeCMD;
import frc.robot.subsystems.IntakeSS;
//import frc.robot.subsystems.WristSS;
 
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
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
    private final ClawSS mClawSS = new ClawSS();
    private final IntakeSS mIntakeSS = new IntakeSS();
    private final ARM_SS mArm_SS = new ARM_SS();
    
    //claw command declaration
    private final ClawExitCMD mClawExitCMD = new ClawExitCMD(mClawSS);
    private final ClawGoAngledCMD mClawGoAngledCMD = new ClawGoAngledCMD(mClawSS);
    private final ClawGoStraight mClawGoStraight = new ClawGoStraight(mClawSS);
    private final ClawIntakeCMD mClawIntakeCMD = new ClawIntakeCMD(mClawSS);

    //intake command declaration
    private final intakeDownCMD mIntakeDownCMD = new intakeDownCMD(mIntakeSS);
    private final intakeUpCMD mIntakeUpCMD = new intakeUpCMD(mIntakeSS);
    private final intakeWheelCMD mIntakeWheelCMD = new intakeWheelCMD(mIntakeSS);
    private final intakeWheelReverseCMD mIntakeWheelReverseCMD = new intakeWheelReverseCMD(mIntakeSS);

    //arm position declaration
    private final goToL1 mGoToL1 = new goToL1(mArm_SS);
    private final goToL2 mGoToL2 = new goToL2(mArm_SS);
    private final goToL3 mGoToL3 = new goToL3(mArm_SS);
    private final goToL4 mGoToL4 = new goToL4(mArm_SS);
    private final goToRest mGoToRest = new goToRest(mArm_SS);

    //command groups declaration
    private final closeRobotCMD mCloseRobotCMD = new closeRobotCMD(mArm_SS, mClawSS);
    private final IntakeInCMD mIntakeInCMD = new IntakeInCMD(mIntakeSS, mArm_SS);
    private final IntakeOutCMD mIntakeOutCMD = new IntakeOutCMD(mIntakeSS, mArm_SS, mClawSS);
 

    private final SendableChooser<Command> autoChooser;



// Xbox Controller
    final CommandXboxController driverXbox = new CommandXboxController(0);

// Console box 
  final GenericHID mButtonBoxPT1 = new GenericHID(2);

    
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
  

  

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/falcon"));

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * -1,
                                                                () -> driverXbox.getLeftX() * -1)
                                                            .withControllerRotationAxis(() -> driverXbox.getRightX() * -0.5)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  SwerveInputStream driveSlowly = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                            () -> driverXbox.getLeftY() * -0.4,
                                                            () -> driverXbox.getLeftX() * -0.4)
                                                        .withControllerRotationAxis(() -> driverXbox.getRightX() * -0.5)
                                                        .deadband(OperatorConstants.DEADBAND)
                                                        .scaleTranslation(0.8)
                                                        .allianceRelativeControl(true);


  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
                                                                                             driverXbox::getRightY)
                                                           .headingWhile(true);

// Red Alliance

SwerveInputStream PoteauAR = driveAngularVelocity.copy().of( drivebase.getSwerveDrive(),
  () -> drivebase.XPose(FieldConstants.RedAlliancePoses.PoteauAR.getX()),
  () -> drivebase.YPose(FieldConstants.RedAlliancePoses.PoteauAR.getY()))                                                            
  .withControllerRotationAxis(() -> drivebase.RotPose(FieldConstants.RedAlliancePoses.PoteauAR.getRotation().getDegrees()));

  SwerveInputStream PoteauBR = driveAngularVelocity.copy().of( drivebase.getSwerveDrive(),
  () -> drivebase.XPose(FieldConstants.RedAlliancePoses.PoteauBR.getX()),
  () -> drivebase.YPose(FieldConstants.RedAlliancePoses.PoteauBR.getY()))                                                            
  .withControllerRotationAxis(() -> drivebase.RotPose(FieldConstants.RedAlliancePoses.PoteauBR.getRotation().getDegrees()));

  SwerveInputStream PoteauCR = driveAngularVelocity.copy().of( drivebase.getSwerveDrive(),
  () -> drivebase.XPose(FieldConstants.RedAlliancePoses.PoteauCR.getX()),
  () -> drivebase.YPose(FieldConstants.RedAlliancePoses.PoteauCR.getY()))                                                            
  .withControllerRotationAxis(() -> drivebase.RotPose(FieldConstants.RedAlliancePoses.PoteauCR.getRotation().getDegrees()));

  SwerveInputStream PoteauDR = driveAngularVelocity.copy().of( drivebase.getSwerveDrive(),
  () -> drivebase.XPose(FieldConstants.RedAlliancePoses.PoteauDR.getX()),
  () -> drivebase.YPose(FieldConstants.RedAlliancePoses.PoteauDR.getY()))                                                            
  .withControllerRotationAxis(() -> drivebase.RotPose(FieldConstants.RedAlliancePoses.PoteauDR.getRotation().getDegrees()));

  SwerveInputStream PoteauER = driveAngularVelocity.copy().of( drivebase.getSwerveDrive(),
  () -> drivebase.XPose(FieldConstants.RedAlliancePoses.PoteauER.getX()),
  () -> drivebase.YPose(FieldConstants.RedAlliancePoses.PoteauER.getY()))                                                            
  .withControllerRotationAxis(() -> drivebase.RotPose(FieldConstants.RedAlliancePoses.PoteauER.getRotation().getDegrees()));

  SwerveInputStream PoteauFR = driveAngularVelocity.copy().of( drivebase.getSwerveDrive(),
  () -> drivebase.XPose(FieldConstants.RedAlliancePoses.PoteauFR.getX()),
  () -> drivebase.YPose(FieldConstants.RedAlliancePoses.PoteauFR.getY()))                                                            
  .withControllerRotationAxis(() -> drivebase.RotPose(FieldConstants.RedAlliancePoses.PoteauFR.getRotation().getDegrees()));

  SwerveInputStream PoteauGR = driveAngularVelocity.copy().of( drivebase.getSwerveDrive(),
  () -> drivebase.XPose(FieldConstants.RedAlliancePoses.PoteauGR.getX()),
  () -> drivebase.YPose(FieldConstants.RedAlliancePoses.PoteauGR.getY()))                                                            
  .withControllerRotationAxis(() -> drivebase.RotPose(FieldConstants.RedAlliancePoses.PoteauGR.getRotation().getDegrees()));

  SwerveInputStream PoteauHR = driveAngularVelocity.copy().of( drivebase.getSwerveDrive(),
  () -> drivebase.XPose(FieldConstants.RedAlliancePoses.PoteauHR.getX()),
  () -> drivebase.YPose(FieldConstants.RedAlliancePoses.PoteauHR.getY()))                                                            
  .withControllerRotationAxis(() -> drivebase.RotPose(FieldConstants.RedAlliancePoses.PoteauHR.getRotation().getDegrees()));

  SwerveInputStream PoteauIR = driveAngularVelocity.copy().of( drivebase.getSwerveDrive(),
  () -> drivebase.XPose(FieldConstants.RedAlliancePoses.PoteauIR.getX()),
  () -> drivebase.YPose(FieldConstants.RedAlliancePoses.PoteauIR.getY()))                                                            
  .withControllerRotationAxis(() -> drivebase.RotPose(FieldConstants.RedAlliancePoses.PoteauIR.getRotation().getDegrees()));

  SwerveInputStream PoteauJR = driveAngularVelocity.copy().of( drivebase.getSwerveDrive(),
  () -> drivebase.XPose(FieldConstants.RedAlliancePoses.PoteauJR.getX()),
  () -> drivebase.YPose(FieldConstants.RedAlliancePoses.PoteauJR.getY()))                                                            
  .withControllerRotationAxis(() -> drivebase.RotPose(FieldConstants.RedAlliancePoses.PoteauJR.getRotation().getDegrees()));

  SwerveInputStream PoteauKR = driveAngularVelocity.copy().of( drivebase.getSwerveDrive(),
  () -> drivebase.XPose(FieldConstants.RedAlliancePoses.PoteauKR.getX()),
  () -> drivebase.YPose(FieldConstants.RedAlliancePoses.PoteauKR.getY()))                                                            
  .withControllerRotationAxis(() -> drivebase.RotPose(FieldConstants.RedAlliancePoses.PoteauKR.getRotation().getDegrees()));

  SwerveInputStream PoteauLR = driveAngularVelocity.copy().of( drivebase.getSwerveDrive(),
  () -> drivebase.XPose(FieldConstants.RedAlliancePoses.PoteauLR.getX()),
  () -> drivebase.YPose(FieldConstants.RedAlliancePoses.PoteauLR.getY()))                                                            
  .withControllerRotationAxis(() -> drivebase.RotPose(FieldConstants.RedAlliancePoses.PoteauLR.getRotation().getDegrees()));

  // Blue Alliance 

  SwerveInputStream PoteauA = driveAngularVelocity.copy().of( drivebase.getSwerveDrive(),
  () -> drivebase.XPose(FieldConstants.BlueAlliancePoses.PoteauA.getX()),
  () -> drivebase.YPose(FieldConstants.BlueAlliancePoses.PoteauA.getY()))                                                            
  .withControllerRotationAxis(() -> drivebase.RotPose(FieldConstants.BlueAlliancePoses.PoteauA.getRotation().getDegrees()));

  SwerveInputStream PoteauB = driveAngularVelocity.copy().of( drivebase.getSwerveDrive(),
  () -> drivebase.XPose(FieldConstants.BlueAlliancePoses.PoteauB.getX()),
  () -> drivebase.YPose(FieldConstants.BlueAlliancePoses.PoteauB.getY()))                                                            
  .withControllerRotationAxis(() -> drivebase.RotPose(FieldConstants.BlueAlliancePoses.PoteauB.getRotation().getDegrees()));

  SwerveInputStream PoteauC = driveAngularVelocity.copy().of( drivebase.getSwerveDrive(),
  () -> drivebase.XPose(FieldConstants.BlueAlliancePoses.PoteauC.getX()),
  () -> drivebase.YPose(FieldConstants.BlueAlliancePoses.PoteauC.getY()))                                                            
  .withControllerRotationAxis(() -> drivebase.RotPose(FieldConstants.BlueAlliancePoses.PoteauC.getRotation().getDegrees()));

  SwerveInputStream PoteauD = driveAngularVelocity.copy().of( drivebase.getSwerveDrive(),
  () -> drivebase.XPose(FieldConstants.BlueAlliancePoses.PoteauD.getX()),
  () -> drivebase.YPose(FieldConstants.BlueAlliancePoses.PoteauD.getY()))                                                            
  .withControllerRotationAxis(() -> drivebase.RotPose(FieldConstants.BlueAlliancePoses.PoteauD.getRotation().getDegrees()));

  SwerveInputStream PoteauE = driveAngularVelocity.copy().of( drivebase.getSwerveDrive(),
  () -> drivebase.XPose(FieldConstants.BlueAlliancePoses.PoteauE.getX()),
  () -> drivebase.YPose(FieldConstants.BlueAlliancePoses.PoteauE.getY()))                                                            
  .withControllerRotationAxis(() -> drivebase.RotPose(FieldConstants.BlueAlliancePoses.PoteauE.getRotation().getDegrees()));

  SwerveInputStream PoteauF = driveAngularVelocity.copy().of( drivebase.getSwerveDrive(),
  () -> drivebase.XPose(FieldConstants.BlueAlliancePoses.PoteauF.getX()),
  () -> drivebase.YPose(FieldConstants.BlueAlliancePoses.PoteauF.getY()))                                                            
  .withControllerRotationAxis(() -> drivebase.RotPose(FieldConstants.BlueAlliancePoses.PoteauF.getRotation().getDegrees()));

  SwerveInputStream PoteauG = driveAngularVelocity.copy().of( drivebase.getSwerveDrive(),
  () -> drivebase.XPose(FieldConstants.BlueAlliancePoses.PoteauG.getX()),
  () -> drivebase.YPose(FieldConstants.BlueAlliancePoses.PoteauG.getY()))                                                            
  .withControllerRotationAxis(() -> drivebase.RotPose(FieldConstants.BlueAlliancePoses.PoteauG.getRotation().getDegrees()));

  SwerveInputStream PoteauH = driveAngularVelocity.copy().of( drivebase.getSwerveDrive(),
  () -> drivebase.XPose(FieldConstants.BlueAlliancePoses.PoteauH.getX()),
  () -> drivebase.YPose(FieldConstants.BlueAlliancePoses.PoteauH.getY()))                                                            
  .withControllerRotationAxis(() -> drivebase.RotPose(FieldConstants.BlueAlliancePoses.PoteauH.getRotation().getDegrees()));

  SwerveInputStream PoteauI = driveAngularVelocity.copy().of( drivebase.getSwerveDrive(),
  () -> drivebase.XPose(FieldConstants.BlueAlliancePoses.PoteauI.getX()),
  () -> drivebase.YPose(FieldConstants.BlueAlliancePoses.PoteauI.getY()))                                                            
  .withControllerRotationAxis(() -> drivebase.RotPose(FieldConstants.BlueAlliancePoses.PoteauI.getRotation().getDegrees()));

  SwerveInputStream PoteauJ = driveAngularVelocity.copy().of( drivebase.getSwerveDrive(),
  () -> drivebase.XPose(FieldConstants.BlueAlliancePoses.PoteauJ.getX()),
  () -> drivebase.YPose(FieldConstants.BlueAlliancePoses.PoteauJ.getY()))                                                            
  .withControllerRotationAxis(() -> drivebase.RotPose(FieldConstants.BlueAlliancePoses.PoteauJ.getRotation().getDegrees()));

  SwerveInputStream PoteauK = driveAngularVelocity.copy().of( drivebase.getSwerveDrive(),
  () -> drivebase.XPose(FieldConstants.BlueAlliancePoses.PoteauK.getX()),
  () -> drivebase.YPose(FieldConstants.BlueAlliancePoses.PoteauK.getY()))                                                            
  .withControllerRotationAxis(() -> drivebase.RotPose(FieldConstants.BlueAlliancePoses.PoteauK.getRotation().getDegrees()));

  SwerveInputStream PoteauL = driveAngularVelocity.copy().of( drivebase.getSwerveDrive(),
  () -> drivebase.XPose(FieldConstants.BlueAlliancePoses.PoteauL.getX()),
  () -> drivebase.YPose(FieldConstants.BlueAlliancePoses.PoteauL.getY()))                                                            
  .withControllerRotationAxis(() -> drivebase.RotPose(FieldConstants.BlueAlliancePoses.PoteauL.getRotation().getDegrees()));
  

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                        () -> -driverXbox.getLeftY(),
                                                                        () -> -driverXbox.getLeftX())
                                                                    .withControllerRotationAxis(() -> driverXbox.getRawAxis(
                                                                        2))
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                               .withControllerHeadingAxis(() ->
                                                                                                              Math.sin(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2))
                                                                               .headingWhile(true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {

   
     
    // Configure the trigger bindings
    DriverStation.silenceJoystickConnectionWarning(true);

    //NamedCommands

    NamedCommands.registerCommand("ClawExitCMD", mClawExitCMD);
    NamedCommands.registerCommand("ClawGoAngled", mClawGoAngledCMD);
    NamedCommands.registerCommand("ClawGoStraight", mClawGoStraight);
    NamedCommands.registerCommand("ClawIntakeCMD", mClawIntakeCMD);

    NamedCommands.registerCommand("IntakeDownCMD", mIntakeDownCMD);
    NamedCommands.registerCommand("IntakeUpCMD", mIntakeUpCMD);
    NamedCommands.registerCommand("IntakeWheelCMD", mIntakeWheelCMD);
    NamedCommands.registerCommand("IntakeWheelReverseCMD", mIntakeWheelReverseCMD);

    NamedCommands.registerCommand("GoToL1CMD", mGoToL1);
    NamedCommands.registerCommand("GoToL2CMD", mGoToL2);
    NamedCommands.registerCommand("GoToL3CMD", mGoToL3);
    NamedCommands.registerCommand("GoToL4CMD", mGoToL4);
    NamedCommands.registerCommand("GoToRest", mGoToRest);

    NamedCommands.registerCommand("CloseRobot", mCloseRobotCMD);
    NamedCommands.registerCommand("IntakeInCMD", mIntakeInCMD);
    NamedCommands.registerCommand("IntakeOutCMD", mIntakeOutCMD);
     

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("WhereWeGoing?", autoChooser);

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

    Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngleKeyboard);

    // Poteau Positions RED
    Command MoveToAR = drivebase.driveFieldOriented(PoteauAR);
    Command MoveToBR = drivebase.driveFieldOriented(PoteauBR);
    Command MoveToCR = drivebase.driveFieldOriented(PoteauCR);
    Command MoveToDR = drivebase.driveFieldOriented(PoteauDR);
    Command MoveToER = drivebase.driveFieldOriented(PoteauER);
    Command MoveToFR = drivebase.driveFieldOriented(PoteauFR);
    Command MoveToGR = drivebase.driveFieldOriented(PoteauGR);
    Command MoveToHR = drivebase.driveFieldOriented(PoteauHR);
    Command MoveToIR = drivebase.driveFieldOriented(PoteauIR);
    Command MoveToJR = drivebase.driveFieldOriented(PoteauJR);
    Command MoveToKR = drivebase.driveFieldOriented(PoteauKR);
    Command MoveToLR = drivebase.driveFieldOriented(PoteauLR);

    // Poteau Positions Bleu

    Command MoveToA = drivebase.driveFieldOriented(PoteauA);
    Command MoveToB = drivebase.driveFieldOriented(PoteauB);
    Command MoveToC = drivebase.driveFieldOriented(PoteauC);
    Command MoveToD = drivebase.driveFieldOriented(PoteauD);
    Command MoveToE = drivebase.driveFieldOriented(PoteauE);
    Command MoveToF = drivebase.driveFieldOriented(PoteauF);
    Command MoveToG = drivebase.driveFieldOriented(PoteauG);
    Command MoveToH = drivebase.driveFieldOriented(PoteauH);
    Command MoveToI = drivebase.driveFieldOriented(PoteauI);
    Command MoveToJ = drivebase.driveFieldOriented(PoteauJ);
    Command MoveToK = drivebase.driveFieldOriented(PoteauK);
    Command MoveToL = drivebase.driveFieldOriented(PoteauL);
    
    //
    Command DriveSlow = drivebase.driveFieldOriented(driveSlowly);


    if (RobotBase.isSimulation())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    } else
    {
      //driveRobotOrientedAngularVelocity
      //driveFieldOrientedAnglularVelocity
      drivebase.setDefaultCommand(driveRobotOrientedAngularVelocity);
    }

    if (Robot.isSimulation()) 
    {
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 5, new Rotation2d()))));
      driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());

    }
    if (DriverStation.isTest())
    {
      
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      driverXbox.leftBumper().onTrue(Commands.none());
      driverXbox.rightBumper().onTrue(Commands.none());
      driverXbox.x().onTrue(drivebase.sysIdDriveMotorCommand());

      driverXbox.y().onTrue(drivebase.sysIdAngleMotorCommand());
       
    } else
    {
      
      //Button BOX
      if(SwerveSubsystem.isRedAlliance() == true) { 
        btn5.whileTrue(MoveToAR);
        btn6.whileTrue(MoveToBR);
        btn7.whileTrue(MoveToCR);
        btn8.whileTrue(MoveToDR);
        btn9.whileTrue(MoveToER);
        btn10.whileTrue(MoveToFR);
        btn11.whileTrue(MoveToGR);
        btn12.whileTrue(MoveToHR);
        btn13.whileTrue(MoveToIR);
        btn14.whileTrue(MoveToJR);
        btn15.whileTrue(MoveToKR);
        btn16.whileTrue(MoveToLR);

      } else if(SwerveSubsystem.isRedAlliance() == false){ 
        btn5.whileTrue(MoveToA);
        btn6.whileTrue(MoveToB);
        btn7.whileTrue(MoveToC);
        btn8.whileTrue(MoveToD);
        btn9.whileTrue(MoveToE);
        btn10.whileTrue(MoveToF);
        btn11.whileTrue(MoveToG);
        btn12.whileTrue(MoveToH);
        btn13.whileTrue(MoveToI);
        btn14.whileTrue(MoveToJ);
        btn15.whileTrue(MoveToK);
        btn16.whileTrue(MoveToL);
 
      } 
      //L1-2-3-4
      //ReefPositions

      driverXbox.start().whileTrue(Commands.none());
      driverXbox.back().whileTrue(Commands.none());
      driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.rightBumper().onTrue(Commands.none());
      driverXbox.a().whileTrue(DriveSlow); 
    }
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
    return autoChooser.getSelected();
    //return drivebase.getAutonomousCommand("goOut");
    //return drivebase.getAutonomousCommand("goStraight");
    //driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}