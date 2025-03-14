// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PositionsDictionnary;


public class 
ARM_SS extends SubsystemBase {
  private final SparkMax mLeadBase;
  private final SparkMax mFollowBase1;
  private final SparkMax mFollowBase2;
  private final SparkMax mFollowBase3;
  private final SparkMax mLeadExtension;
  private final SparkMax mFollowExtension;
  private final SparkMax mWristMotor;

  private final RelativeEncoder mExtensionEncoder;
  private final RelativeEncoder mArmEncoder;
  private final RelativeEncoder mWristEncoder;

  private final SparkClosedLoopController mExtensionPIDController;
  private final SparkClosedLoopController mArmPIDControler;
  private final SparkClosedLoopController mWristPIDController;

  private final SparkMaxConfig mLeadConfig;
  private final SparkMaxConfig mFollowConfig1;
  private final SparkMaxConfig mFollowConfig2;
  private final SparkMaxConfig mLeadExtensionConfig;
  private final SparkMaxConfig mFollowExtensionConfig;
  private final SparkMaxConfig mWristConfig;

  private boolean currentlyRunning = false;
  public  boolean done = false;
  public PositionType_SS currentPostion = PositionsDictionnary.mPositionRest;

  /** Creates a new ARM_SS. */
  public ARM_SS() {
    //ARM MOTORS
    mLeadBase = new SparkMax(11,MotorType.kBrushless);
    mLeadConfig = new SparkMaxConfig();
      mLeadConfig
      .inverted(false)
      .idleMode(IdleMode.kBrake);
      mLeadConfig.alternateEncoder
      .positionConversionFactor(Constants.ArmConstants.RotationdegresParTour)
      .velocityConversionFactor(Constants.ArmConstants.RotationdegresParTour)
      .inverted(true);
      mLeadConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
      .p(0.025,ClosedLoopSlot.kSlot0)
      .i(0,ClosedLoopSlot.kSlot0)
      .d(0,ClosedLoopSlot.kSlot0)
      .minOutput(-0.3)
      .maxOutput(0.3)
      ;
    mLeadBase.configure(mLeadConfig,ResetMode.kNoResetSafeParameters,PersistMode.kNoPersistParameters);

    mFollowBase1 = new SparkMax(12,MotorType.kBrushless);
    mFollowBase2 = new SparkMax(13, MotorType.kBrushless);
    mFollowBase3 = new SparkMax(14, MotorType.kBrushless);
      mFollowConfig1 = new SparkMaxConfig();
      mFollowConfig1.follow(mLeadBase);
      mFollowConfig2 = new SparkMaxConfig();
      mFollowConfig2.follow(mLeadBase,true);
    mFollowBase1.configure(mFollowConfig1,ResetMode.kNoResetSafeParameters,PersistMode.kNoPersistParameters);
    mFollowBase2.configure(mFollowConfig2,ResetMode.kNoResetSafeParameters,PersistMode.kNoPersistParameters);
    mFollowBase3.configure(mFollowConfig2,ResetMode.kNoResetSafeParameters,PersistMode.kNoPersistParameters);

    //EXTENSION MOTORS
    mLeadExtension = new SparkMax(15,MotorType.kBrushless);
    mFollowExtension = new SparkMax(16, MotorType.kBrushless);

    mLeadExtensionConfig = new SparkMaxConfig();
      mLeadExtensionConfig.idleMode(IdleMode.kBrake)
      .inverted(false);
      mLeadExtensionConfig.alternateEncoder
      .positionConversionFactor(Constants.ArmConstants.ExtensionPouceParTour)
      .velocityConversionFactor(Constants.ArmConstants.ExtensionPouceParTour)
      .inverted(true);
      mLeadExtensionConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
      .pid(0.07, 0.00005, 0.00001, ClosedLoopSlot.kSlot0)
      .minOutput(-0.20)
      .maxOutput(0.20);
    mLeadExtension.configure(mLeadExtensionConfig,ResetMode.kNoResetSafeParameters,PersistMode.kNoPersistParameters);

      
    mFollowExtensionConfig = new SparkMaxConfig();
      mFollowExtensionConfig.follow(mLeadExtension,false);
  
    mFollowExtension.configure(mFollowExtensionConfig,ResetMode.kNoResetSafeParameters,PersistMode.kNoPersistParameters);

    //WRIST MOTOR
    mWristMotor = new SparkMax(17,MotorType.kBrushless);
      mWristConfig = new SparkMaxConfig();
        mWristConfig.idleMode(IdleMode.kBrake);
      mWristConfig.alternateEncoder
        .positionConversionFactor(Constants.ArmConstants.WristdegresParTour)
        .velocityConversionFactor(1)
        .inverted(true);
      mWristConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
      .pid(0.007,0,0,ClosedLoopSlot.kSlot0)
      .minOutput(-0.25)
      .maxOutput(0.25);
    mWristMotor.configure(mWristConfig,ResetMode.kNoResetSafeParameters,PersistMode.kNoPersistParameters);
      
    mExtensionEncoder = mLeadExtension.getAlternateEncoder();
    mArmEncoder = mLeadBase.getAlternateEncoder();
    mWristEncoder = mWristMotor.getAlternateEncoder();

    mExtensionPIDController = mLeadExtension.getClosedLoopController();
    mArmPIDControler = mLeadBase.getClosedLoopController();
    mWristPIDController = mWristMotor.getClosedLoopController();

    SmartDashboard.putNumber("extensionSetpoint", 0);
    SmartDashboard.putNumber("rotationSetpoint", 0);
    SmartDashboard.putNumber("WristSetpoint", 0);

  }

 /*  public void ExtensionGoToPosition(){
    mExtensionPIDController.setReference( SmartDashboard.getNumber("extensionSetpoint", 0), ControlType.kPosition,ClosedLoopSlot.kSlot0);
  }

  public void RotationGoToPosition(){    
    mArmPIDControler.setReference(SmartDashboard.getNumber("rotationSetpoint", 0) + Constants.ArmConstants.RotationEncoderOffSet, ControlType.kPosition,ClosedLoopSlot.kSlot0);
  }

  public void WristGoToPosition(){
    mExtensionPIDController.setReference(SmartDashboard.getNumber("WristSetpoint", 0), ControlType.kPosition,ClosedLoopSlot.kSlot0);
  }
  */
 // public void AllInOne(/*double pLongueur, double ppPosition.armAngle*//*, parmAngleWrist */){
 //   RotationGoToPosition();
 //   ExtensionGoToPosition();
 //   WristGoToPosition();
 //}

 /*  public void restart(){
  // currentlyRunning = false;
   //done = false;
  }
*/
  public boolean isDone(){
    return done;
  }
/* 
  public void change_position_3steps(double armAngle, double longueur, double threshold){//threshold is not used curently but might be usefull
    
    if(currentlyRunning == false){
      
      mExtensionPIDController.setReference(0, ControlType.kPosition,ClosedLoopSlot.kSlot0);
      currentlyRunning = true;
    }
    else if(currentlyRunning==true){
            if (mExtensionEncoder.getPosition() <= 0 + 2 ){
                mArmPIDControler.setReference(armAngle + Constants.ArmConstants.RotationEncoderOffSet, ControlType.kPosition,ClosedLoopSlot.kSlot0);
              }
            if(mArmEncoder.getPosition() - Constants.ArmConstants.RotationEncoderOffSet >= armAngle - 10 && mArmEncoder.getPosition() - Constants.ArmConstants.RotationEncoderOffSet <= armAngle + 10){
                mExtensionPIDController.setReference(longueur, ControlType.kPosition,ClosedLoopSlot.kSlot0);
                //currentlyRunning = false;
                if ((mExtensionEncoder.getPosition() >= longueur-1) && (mExtensionEncoder.getPosition() <= longueur + 1) ){
                  done = true;
                  currentlyRunning = false;
                }
              }

      }
    }
        */

    public double getArmPosition(){
      return mArmEncoder.getPosition() - Constants.ArmConstants.RotationEncoderOffSet;
    }
    public double getExtensionPosition(){
      return mExtensionEncoder.getPosition()- Constants.ArmConstants.ExtensionEncoderOffSet;
    }
    public double getWristPosition(){
      return mWristEncoder.getPosition() - Constants.ArmConstants.WristEncoderOffSet;
    }



    public boolean isArmInPosition (double wantedarmAngle,double tolerance){
      return (mArmEncoder.getPosition() - Constants.ArmConstants.RotationEncoderOffSet >= wantedarmAngle - tolerance && mArmEncoder.getPosition() - Constants.ArmConstants.RotationEncoderOffSet <= wantedarmAngle + tolerance);
    }
    public boolean isLenghtInPostition(double wantedArmLength,double tolerance){
      return (mExtensionEncoder.getPosition() >= wantedArmLength - tolerance && mExtensionEncoder.getPosition() <= wantedArmLength + tolerance);
    }
    public boolean isWristInPosition(double wantedarmAngle,double tolerance){
      return (mWristEncoder.getPosition() - Constants.ArmConstants.WristEncoderOffSet >= wantedarmAngle - tolerance && mWristEncoder.getPosition() - Constants.ArmConstants.WristEncoderOffSet <= wantedarmAngle + tolerance);
    }
//old way of doing it
   /*  public void strategie1A(double pPosition.armAngle,double pPosition.armTolerance, double pPosition.armLength,double pPosition.lenghtTolerance,double pPosition.wrist ,double pPosition.wristTolerance,double rotationWrist){
      if(currentlyRunning == false){
        mArmPIDControler.setReference(pPosition.armAngle + Constants.ArmConstants.RotationEncoderOffSet, ControlType.kPosition,ClosedLoopSlot.kSlot0);
        currentlyRunning = true;
      }
      else if(currentlyRunning==true){
              if (isArmInPosition(pPosition.armAngle, pPosition.armTolerance)){
                  mWristPIDController.setReference(pPosition.wrist + Constants.ArmConstants.WristEncoderOffRotationEncoderOffSet, ControlType.kPosition,ClosedLoopSlot.kSlot0);
                }
              if(isWristInPosition(pPosition.wrist,pPosition.wristTolerance)){
                  mExtensionPIDController.setReference(pPosition.armLength, ControlType.kPosition,ClosedLoopSlot.kSlot0);
                  
                   if (isLenghtInPostition(pPosition.armLength, pPosition.wristTolerance) ){
                    done = true;
                    currentlyRunning = false;
                  }
                }
  
        }
      }
*/


        //arm -> wrist -> longueur 
      public void S1A(PositionType_SS pPosition){
        SmartDashboard.putString("order","arm -> wrist -> longueur");
        SmartDashboard.putString("target",pPosition.name);
        SmartDashboard.putString("strat","S1A");

        if(!isArmInPosition(pPosition.armAngle, pPosition.armTolerance) && !isLenghtInPostition(pPosition.armLength, pPosition.lenghtTolerance) && !isWristInPosition(pPosition.wrist, pPosition.wristTolerance)){
          SmartDashboard.putString("step"," arm -> wrist");
          mArmPIDControler.setReference(pPosition.armAngle + Constants.ArmConstants.RotationEncoderOffSet, ControlType.kPosition,ClosedLoopSlot.kSlot0);
        }

           if (isArmInPosition(pPosition.armAngle, pPosition.armTolerance) && isWristInPosition(pPosition.wrist,pPosition.wristTolerance) && isLenghtInPostition(pPosition.armLength, pPosition.lenghtTolerance)){
            SmartDashboard.putString("step"," done");
            done = true;
            }
             if (isArmInPosition(pPosition.armAngle, pPosition.armTolerance) && isWristInPosition(pPosition.wrist, pPosition.wristTolerance)&& !isLenghtInPostition(pPosition.armLength, pPosition.lenghtTolerance)){
              SmartDashboard.putString("step"," lenght -> done");
              mExtensionPIDController.setReference(pPosition.armLength, ControlType.kPosition,ClosedLoopSlot.kSlot0);
              }
               if(isArmInPosition(pPosition.armAngle, pPosition.armTolerance) && !isWristInPosition(pPosition.wrist, pPosition.wristTolerance) && !isLenghtInPostition(pPosition.armLength, pPosition.lenghtTolerance)){
                SmartDashboard.putString("step"," wrist -> lenght");
                mWristPIDController.setReference(pPosition.wrist + Constants.ArmConstants.WristEncoderOffSet, ControlType.kPosition,ClosedLoopSlot.kSlot0);
                }
    
          }
        

        //armAngle -> wrist + longueur
      public void S1B(PositionType_SS pPosition){
        
        SmartDashboard.putString("order","armAngle -> wrist + longueur");  
        SmartDashboard.putString("target",pPosition.name);
        SmartDashboard.putString("strat","S1B");

        if(!isArmInPosition(pPosition.armAngle, pPosition.armTolerance) && !isLenghtInPostition(pPosition.armLength, pPosition.lenghtTolerance) && !isWristInPosition(pPosition.wrist, pPosition.wristTolerance)){
          SmartDashboard.putString("step"," arm -> wrist + extension");
            mArmPIDControler.setReference(pPosition.armAngle + Constants.ArmConstants.RotationEncoderOffSet, ControlType.kPosition,ClosedLoopSlot.kSlot0);
          }
            //is everything in position
            if (isArmInPosition(pPosition.armAngle, pPosition.armTolerance) && isWristInPosition(pPosition.wrist,pPosition.wristTolerance) && isLenghtInPostition(pPosition.armLength, pPosition.lenghtTolerance)){
              SmartDashboard.putString("step","done");
              done = true;
              }
              //if arm is in position start wrist and pPosition.armLength
               if(isArmInPosition(pPosition.armAngle, pPosition.armTolerance) && !isWristInPosition(pPosition.wrist, pPosition.wristTolerance) && !isLenghtInPostition(pPosition.armLength, pPosition.lenghtTolerance)){
                SmartDashboard.putString("step"," wrist + extension -> done");
                mWristPIDController.setReference(pPosition.wrist + Constants.ArmConstants.WristEncoderOffSet, ControlType.kPosition,ClosedLoopSlot.kSlot0);
                mExtensionPIDController.setReference(pPosition.armLength, ControlType.kPosition,ClosedLoopSlot.kSlot0);
                }
            }
                
            
          
        //wrist -> longueur + armAngle
      public void S2B(PositionType_SS pPosition){
        SmartDashboard.putString("order","wrist -> longueur + armAngle"); 
        SmartDashboard.putString("target",pPosition.name);
        SmartDashboard.putString("strat","S2B");
 
        if(!isArmInPosition(pPosition.armAngle, pPosition.armTolerance) && !isLenghtInPostition(pPosition.armLength, pPosition.lenghtTolerance) && !isWristInPosition(pPosition.wrist, pPosition.wristTolerance)){
          //start wrist
          SmartDashboard.putString("step"," wrist -> arm + extension");
          mWristPIDController.setReference(pPosition.wrist + Constants.ArmConstants.WristEncoderOffSet, ControlType.kPosition,ClosedLoopSlot.kSlot0);
        }
          //is everything in position
          if (isArmInPosition(pPosition.armAngle, pPosition.armTolerance) && isWristInPosition(pPosition.wrist,pPosition.wristTolerance) && isLenghtInPostition(pPosition.armLength, pPosition.lenghtTolerance)){
            SmartDashboard.putString("step","done");
            done = true;
            } 
            // if wrist is in position start pPosition.armLength and armAngle
             if(isWristInPosition(pPosition.wrist, pPosition.wristTolerance) && !isLenghtInPostition(pPosition.armLength, pPosition.lenghtTolerance) && !isArmInPosition(pPosition.armAngle, pPosition.armTolerance)){
            SmartDashboard.putString("step"," arm + extension -> done");
            mArmPIDControler.setReference(pPosition.armAngle + Constants.ArmConstants.RotationEncoderOffSet, ControlType.kPosition,ClosedLoopSlot.kSlot0);
            mExtensionPIDController.setReference(pPosition.armLength, ControlType.kPosition,ClosedLoopSlot.kSlot0);
            }
        }
      

        //longueur + wrist -> armAngle
      public void S3(PositionType_SS pPosition){
        SmartDashboard.putString("order","longueur + wrist -> armAngle");  
        SmartDashboard.putString("target",pPosition.name);
        SmartDashboard.putString("strat","S3");

        if(!isArmInPosition(pPosition.armAngle, pPosition.armTolerance) && !isLenghtInPostition(pPosition.armLength, pPosition.lenghtTolerance) && !isWristInPosition(pPosition.wrist, pPosition.wristTolerance)){
          SmartDashboard.putString("step"," wrist + extension -> arm ");

            mExtensionPIDController.setReference(pPosition.armLength , ControlType.kPosition,ClosedLoopSlot.kSlot0);
            mWristPIDController.setReference(pPosition.wrist + Constants.ArmConstants.WristEncoderOffSet, ControlType.kPosition,ClosedLoopSlot.kSlot0);
          }
            //is everything in position
            if (isArmInPosition(pPosition.armAngle, pPosition.armTolerance) && isWristInPosition(pPosition.wrist,pPosition.wristTolerance) && isLenghtInPostition(pPosition.armLength, pPosition.lenghtTolerance)){
              SmartDashboard.putString("step","done");
              done = true;
              }
               if(isLenghtInPostition(pPosition.armLength, pPosition.lenghtTolerance) && isWristInPosition(pPosition.wrist, pPosition.wristTolerance)&& !isArmInPosition(pPosition.armAngle, pPosition.armTolerance)){
                SmartDashboard.putString("step"," arm -> done");
                mArmPIDControler.setReference(pPosition.armAngle + Constants.ArmConstants.RotationEncoderOffSet, ControlType.kPosition,ClosedLoopSlot.kSlot0);
                }
             }
              

        //armAngle -> longueur -> wrist
      public void S4(PositionType_SS pPosition){
        SmartDashboard.putString("order","armAngle -> longueur -> wrist"); 
        SmartDashboard.putString("target",pPosition.name);
        SmartDashboard.putString("strat","S4");
 
        
        if(!isArmInPosition(pPosition.armAngle, pPosition.armTolerance) && !isLenghtInPostition(pPosition.armLength, pPosition.lenghtTolerance) && !isWristInPosition(pPosition.wrist, pPosition.wristTolerance)){
          SmartDashboard.putString("step"," arm -> extension ");

          mArmPIDControler.setReference(pPosition.armAngle + Constants.ArmConstants.RotationEncoderOffSet, ControlType.kPosition,ClosedLoopSlot.kSlot0);
        }
         if (isArmInPosition(pPosition.armAngle, pPosition.armTolerance) && isWristInPosition(pPosition.wrist,pPosition.wristTolerance) && isLenghtInPostition(pPosition.armLength, pPosition.lenghtTolerance)){
            SmartDashboard.putString("step","done");

            done = true;
            }
             if (isArmInPosition(pPosition.armAngle, pPosition.armTolerance) && isLenghtInPostition(pPosition.armLength, pPosition.lenghtTolerance)&& !isWristInPosition(pPosition.wrist, pPosition.wristTolerance)){
              SmartDashboard.putString("step","wrist -> done");

              mWristPIDController.setReference(pPosition.armLength, ControlType.kPosition,ClosedLoopSlot.kSlot0);
              }
               if(isArmInPosition(pPosition.armAngle, pPosition.armTolerance)){
                SmartDashboard.putString("step","extension -> wrist");

                mExtensionPIDController.setReference(pPosition.armLength , ControlType.kPosition,ClosedLoopSlot.kSlot0);
                }
    
          }
        
      
 
      public PositionType_SS whereAmI (){
        return currentPostion; 
      }
    
      public void imHere(PositionType_SS pPostion){
        currentPostion = pPostion;
      }
      


  public void ManualExtension(double pSpeed){
    mLeadExtension.set(pSpeed);
  }
  
  public void ManualRotation(double pSpeed){
    mLeadBase.set(pSpeed);
  }

  /*public void ManualWrist(double pSpeed){
    mWrist.set(pSpeed);
  }*/

  public void stopExtension(){
    mLeadExtension.set(0);
  }

  public void stopRotation(){
    mLeadBase.set(0);
  }

  /*public void stopWrist(){
    mWrist.set(0);
  }*/
  @Override
  public void periodic() {
    SmartDashboard.putNumber("actual Extension position", getExtensionPosition());
    SmartDashboard.putNumber("actual rotation position", getArmPosition());
    SmartDashboard.putNumber("actual Wrist Position", getWristPosition());

    SmartDashboard.putNumber("Extension output", mLeadExtension.getAppliedOutput());
    SmartDashboard.putNumber("Wrist output", mWristMotor.getAppliedOutput());
    SmartDashboard.putNumber("Rotation output", mLeadBase.getAppliedOutput());

    SmartDashboard.putBoolean("done", done);
    SmartDashboard.putString("currentPostion",currentPostion.name);
  }
}
