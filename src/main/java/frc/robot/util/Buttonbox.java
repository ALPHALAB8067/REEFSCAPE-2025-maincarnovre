// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button. Trigger;



/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public public 0static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public  class Buttonbox {  
  public static final GenericHID mButtonBoxPT1 = new GenericHID(2);
  public static final GenericHID mButtonBoxPT2 = new GenericHID(3);
  public static Trigger btn1 = new  Trigger(()->mButtonBoxPT1.getRawButton(1));
  public static Trigger btn2 = new  Trigger(()->mButtonBoxPT1.getRawButton(2));
  public static Trigger btn3 = new  Trigger(()->mButtonBoxPT1.getRawButton(3));
  public static Trigger btn4 = new  Trigger(()->mButtonBoxPT1.getRawButton(4));
  public static Trigger btn5 = new  Trigger(()->mButtonBoxPT1.getRawButton(5));
  public static Trigger btn6 = new  Trigger(()->mButtonBoxPT1.getRawButton(6));
  public static Trigger btn7 = new  Trigger(()->mButtonBoxPT1.getRawButton(7));
  public static Trigger btn8 = new  Trigger(()->mButtonBoxPT1.getRawButton(8));
  //nine is the shift button
  public static Trigger btn10 = new Trigger(()->mButtonBoxPT1.getRawButton(10));
  public static Trigger btn11 = new Trigger(()->mButtonBoxPT1.getRawButton(11));
  public static Trigger btn12 = new Trigger(()->mButtonBoxPT1.getRawButton(12));
  public static Trigger shifted_btn1 = new Trigger(()->mButtonBoxPT1.getRawButton(17));


//reef 
  public static Trigger other_btn1 = new   Trigger(()->mButtonBoxPT2.getRawButton(1));
  public static Trigger other_btn2 = new   Trigger(()->mButtonBoxPT2.getRawButton(2));
  public static Trigger other_btn3 = new   Trigger(()->mButtonBoxPT2.getRawButton(3));
  public static Trigger other_btn4 = new   Trigger(()->mButtonBoxPT2.getRawButton(4));
  public static Trigger other_btn5 = new   Trigger(()->mButtonBoxPT2.getRawButton(5));
  public static Trigger other_btn6 = new   Trigger(()->mButtonBoxPT2.getRawButton(6));
  public static Trigger other_btn7 = new   Trigger(()->mButtonBoxPT2.getRawButton(7));
  public static Trigger other_btn8 = new   Trigger(()->mButtonBoxPT2.getRawButton(8));
  //nine is the shift button
  public static Trigger other_btn10 = new  Trigger(()->mButtonBoxPT2.getRawButton(10));
  public static Trigger other_btn11 = new  Trigger(()->mButtonBoxPT2.getRawButton(11));
  public static Trigger other_btn12 = new  Trigger(()->mButtonBoxPT2.getRawButton(12));





}