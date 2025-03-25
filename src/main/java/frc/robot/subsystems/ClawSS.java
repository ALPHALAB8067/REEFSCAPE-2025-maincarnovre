package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.ClawConstants;

public class ClawSS extends SubsystemBase {

  private final SparkMax rotateSparkMax;
  private final SparkMax wheelSparkMax;

  private final SparkMaxConfig rotateConfig;
  private final SparkMaxConfig wheelConfig;

  private final RelativeEncoder thru;

  private final SparkClosedLoopController rotatePID;  

  public ClawSS() {

    rotateSparkMax = new SparkMax(ClawConstants.rotateSparkMaxPort, MotorType.kBrushless);

    rotateConfig = new SparkMaxConfig();
    rotateConfig.inverted(ClawConstants.rotateIsInverted);
    rotateConfig.idleMode(ClawConstants.rotateIdleMode);
    rotateConfig.encoder.positionConversionFactor(ClawConstants.positionConversionFactor);
    rotateConfig.closedLoop
    .feedbackSensor(ClawConstants.feedbacksensor)
    .p(ClawConstants.pValue)
    .i(ClawConstants.iValue)
    .d(ClawConstants.dValue);

    wheelSparkMax = new SparkMax(ClawConstants.wheelSparkMaxPort, MotorType.kBrushless);

    wheelConfig = new SparkMaxConfig();
    wheelConfig.inverted(ClawConstants.wheelIsInverted);
    wheelConfig.idleMode(ClawConstants.wheelIdleMode);

    thru = rotateSparkMax.getEncoder();

    rotatePID = rotateSparkMax.getClosedLoopController();


  }

  public double getRotatePosition() {
    return thru.getPosition();
  }

  public void goToStraight(double position) {
    rotatePID.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  public void goToAngled(double position) {
    rotatePID.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  public void stopWrist() {
    rotateSparkMax.set(0);
  }

  public boolean IsAtAngle(double angle){
    return (thru.getPosition()>= angle-3 && thru.getPosition()<= angle+3);
  }

  public void turnWheel() {
    wheelSparkMax.set(ClawConstants.turnWheelPercent);
  }

  public void reverseWheel() {
    wheelSparkMax.set(ClawConstants.reverseWheelPercent);
  }

  public void stop() {
    wheelSparkMax.stopMotor();
  }
  

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("IsAtAngle", IsAtAngle(90));
    SmartDashboard.putNumber("Position", getRotatePosition());

  }
}
