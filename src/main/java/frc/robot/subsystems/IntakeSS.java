package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.AnalogInput;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.intakeConstants;

public class IntakeSS extends SubsystemBase {

  @SuppressWarnings("unused")
  private final PneumaticHub ph;
  private final DoubleSolenoid leftSolenoid;
  private final DoubleSolenoid rightSolenoid;

  private final Compressor compressor;

  private final SparkMax wheelSparkMax;

  private final DigitalInput icanseeyou;

  public IntakeSS() {

    ph = new PneumaticHub(intakeConstants.PneumaticHubPort);
    leftSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, intakeConstants.leftSolenoidFWDPort, intakeConstants.leftSolenoidRVRSPort);
    rightSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, intakeConstants.rightSolenoidFWDPort, intakeConstants.rightSolenoidRVRSPort);
    
    compressor = new Compressor(PneumaticsModuleType.REVPH);
    compressor.enableAnalog(105, 120);

    wheelSparkMax = new SparkMax(intakeConstants.wheelSparkMaxPort, MotorType.kBrushless);

    icanseeyou = new DigitalInput(0);


  }



  public void intakeUp() {
    leftSolenoid.set(Value.kForward);
    rightSolenoid.set(Value.kForward);
  }

  public void intakeDown() {
    leftSolenoid.set(Value.kReverse);
    rightSolenoid.set(Value.kReverse);
  }

  public Value getLeftIntakeState() {
    return leftSolenoid.get();
  }

  public Value getRightIntakeState() {
    return rightSolenoid.get();
  }

  public void stop() {
    leftSolenoid.set(Value.kOff);
    rightSolenoid.set(Value.kOff);
  }

  public void wheelTurn() {
    wheelSparkMax.set(intakeConstants.wheelTurnPercent);
  }

  public void wheelReverse() {
    wheelSparkMax.set(intakeConstants.wheelReversePercent);
  }

  
  public double getPressure() {
    return compressor.getPressure();
  }

  public boolean doISeeYou() {
    return icanseeyou.get();
  }
  

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Pressure", getPressure());
    SmartDashboard.putString("Left Intake State", getLeftIntakeState().toString());
    SmartDashboard.putString("Right Intake State", getRightIntakeState().toString());

  }
}
