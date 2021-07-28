/**
 * Firmata.java - Firmata library for Java
 * Copyright (C) 2006-13 David A. Mellis
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General
 * Public License along with this library; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place, Suite 330,
 * Boston, MA  02111-1307  USA
 *
 * Java code to communicate with the Arduino Firmata 2 firmware.
 * http://firmata.org/
 *
 * $Id$
 */

package org.firmata; // hope this is okay!

/**
 * Internal class used by the Arduino class to parse the Firmata protocol.
 */
public class Firmata {
  /**
   * Constant to set a pin to input mode (in a call to pinMode()).
   */
  public static final int INPUT = 0;
  /**
   * Constant to set a pin to output mode (in a call to pinMode()).
   */
  public static final int OUTPUT = 1;
  /**
   * Constant to set a pin to analog mode (in a call to pinMode()).
   */
  public static final int ANALOG = 2;
  /**
   * Constant to set a pin to PWM mode (in a call to pinMode()).
   */
  public static final int PWM = 3;
  /**
   * Constant to set a pin to servo mode (in a call to pinMode()).
   */
  public static final int SERVO = 4;
  /**
   * Constant to set a pin to shiftIn/shiftOut mode (in a call to pinMode()).
   */
  public static final int SHIFT = 5;
  /**
   * Constant to set a pin to I2C mode (in a call to pinMode()).
   */
  public static final int I2C = 6;

  /**
   * Constant to write a high value (+5 volts) to a pin (in a call to
   * digitalWrite()).
   */
  public static final int LOW = 0;
  /**
   * Constant to write a low value (0 volts) to a pin (in a call to
   * digitalWrite()).
   */
  public static final int HIGH = 1;

  private final int MAX_DATA_BYTES = 4096;

  private final int DIGITAL_MESSAGE        = 0x90; // send data for a digital port
  private final int ANALOG_MESSAGE         = 0xE0; // send data for an analog pin (or PWM)
  private final int REPORT_ANALOG          = 0xC0; // enable analog input by pin #
  private final int REPORT_DIGITAL         = 0xD0; // enable digital input by port
  private final int SET_PIN_MODE           = 0xF4; // set a pin to INPUT/OUTPUT/PWM/etc
  private final int REPORT_VERSION         = 0xF9; // report firmware version
  private final int SYSTEM_RESET           = 0xFF; // reset from MIDI
  private final int START_SYSEX            = 0xF0; // start a MIDI SysEx message
  private final int END_SYSEX              = 0xF7; // end a MIDI SysEx message

  // extended command set using sysex (0-127/0x00-0x7F)
  /* 0x00-0x0F reserved for user-defined commands */
  private final int ACCELSTEPPER_DATA           = 0x62; //
  private final int ACCELSTEPPER_ZERO           = 0x01; //
  private final int ACCELSTEPPER_STEP           = 0x02; //
  private final int ACCELSTEPPER_CONFIG         = 0x00; //
  private final int ACCELSTEPPER_TO             = 0x03; //
  private final int ACCELSTEPPER_ENABLE         = 0x04; //
  private final int ACCELSTEPPER_STOP           = 0x05; //
  private final int ACCELSTEPPER_REPORT  		= 0x06; //
  private final int ACCELSTEPPER_MOVECOMPLETE   = 0x0A; //
  private final int ACCELSTEPPER_SETACCEL       = 0x08; //
  private final int ACCELSTEPPER_SETSPEED       = 0x09; //
  private final int ACCELSTEPPER_MULTICONF      = 0x20; //
  private final int ACCELSTEPPER_MULTI_TO       = 0x21; //
  private final int ACCELSTEPPER_MULTI_STOP     = 0x23; //
  private final int ACCELSTEPPER_MULTI_MOVECOMPLETE       = 0x24; //

  
  private final int SERVO_CONFIG           = 0x70; // set max angle, minPulse, maxPulse, freq
  private final int STRING_DATA            = 0x71; // a string message with 14-bits per char
  private final int SHIFT_DATA             = 0x75; // a bitstream to/from a shift register
  private final int I2C_REQUEST            = 0x76; // send an I2C read/write request
  private final int I2C_REPLY              = 0x77; // a reply to an I2C read request
  private final int I2C_CONFIG             = 0x78; // config I2C settings such as delay times and power pins
  private final int EXTENDED_ANALOG        = 0x6F; // analog write (PWM, Servo, etc) to any pin
  private final int PIN_STATE_QUERY        = 0x6D; // ask for a pin's current mode and value
  private final int PIN_STATE_RESPONSE     = 0x6E; // reply with pin's current mode and value
  private final int CAPABILITY_QUERY       = 0x6B; // ask for supported modes and resolution of all pins
  private final int CAPABILITY_RESPONSE    = 0x6C; // reply with supported modes and resolution
  private final int ANALOG_MAPPING_QUERY   = 0x69; // ask for mapping of analog to pin numbers
  private final int ANALOG_MAPPING_RESPONSE= 0x6A; // reply with mapping info
  private final int REPORT_FIRMWARE        = 0x79; // report name and version of the firmware
  private final int SAMPLING_INTERVAL      = 0x7A; // set the poll rate of the main loop
  private final int SYSEX_NON_REALTIME     = 0x7E; // MIDI Reserved for non-realtime messages
  private final int SYSEX_REALTIME         = 0x7F; // MIDI Reserved for realtime messages

  int waitForData = 0;
  int executeMultiByteCommand = 0;
  int multiByteChannel = 0;
  int[] storedInputData = new int[MAX_DATA_BYTES];
  boolean parsingSysex;
  int sysexBytesRead;

  int[] digitalOutputData = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  int[] digitalInputData  = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  int[] analogInputData   = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  private final int MAX_PINS = 128;

  int[] pinModes = new int[MAX_PINS];
  int[] analogChannel = new int[MAX_PINS];
  int[] accelStepperChannel = new int[8];
  int[] pinMode = new int[MAX_PINS];

  int majorVersion = 0;
  int minorVersion = 0;

  /**
   * An interface that the Firmata class uses to write output to the Arduino
   * board. The implementation should forward the data over the actual
   * connection to the board.
   */
  public interface Writer {
    /**
     * Write a byte to the Arduino board. The implementation should forward
     * this using the actual connection.
     *
     * @param val the byte to write to the Arduino board
     */
    public void write(int val);
  }

  Writer out;

  /**
   * Create a proxy to an Arduino board running the Firmata 2 firmware.
   *
   * @param writer an instance of the Firmata.Writer interface
   */
  public Firmata(Writer writer) {
    this.out = writer;
  }

  public void init() {
    // enable all ports; firmware should ignore non-existent ones
    for (int i = 0; i < 16; i++) {
      out.write(REPORT_DIGITAL | i);
      out.write(1);
    }

    //queryCapabilities();
    queryAnalogMapping();

//    for (int i = 0; i < 16; i++) {
//      out.write(REPORT_ANALOG | i);
//      out.write(1);
//    }
  }

  /**
   * Returns the last known value read from the digital pin: HIGH or LOW.
   *
   * @param pin the digital pin whose value should be returned (from 2 to 13,
   * since pins 0 and 1 are used for serial communication)
   */
  public int digitalRead(int pin) {
    return (digitalInputData[pin >> 3] >> (pin & 0x07)) & 0x01;
  }

  /**
   * Returns the last known value read from the analog pin: 0 (0 volts) to
   * 1023 (5 volts).
   *
   * @param pin the analog pin whose value should be returned (from 0 to 5)
   */
  public int analogRead(int pin) {
    return analogInputData[pin];
  }

  /**
   * Set a digital pin to input or output mode.
   *
   * @param pin the pin whose mode to set (from 2 to 13)
   * @param mode either Arduino.INPUT or Arduino.OUTPUT
   */
  public void pinMode(int pin, int mode) {
    out.write(SET_PIN_MODE);
    out.write(pin);
    out.write(mode);
  }

  /**
   * Write to a digital pin (the pin must have been put into output mode with
   * pinMode()).
   *
   * @param pin the pin to write to (from 2 to 13)
   * @param value the value to write: Arduino.LOW (0 volts) or Arduino.HIGH
   * (5 volts)
   */
  public void digitalWrite(int pin, int value) {
    int portNumber = (pin >> 3) & 0x0F;

    if (value == 0)
      digitalOutputData[portNumber] &= ~(1 << (pin & 0x07));
    else
      digitalOutputData[portNumber] |= (1 << (pin & 0x07));

    out.write(DIGITAL_MESSAGE | portNumber);
    out.write(digitalOutputData[portNumber] & 0x7F);
    out.write(digitalOutputData[portNumber] >> 7);
  }

  /**
   * Write an analog value (PWM-wave) to a digital pin.
   *
   * @param pin the pin to write to (must be 9, 10, or 11, as those are they
   * only ones which support hardware pwm)
   * @param value the value: 0 being the lowest (always off), and 255 the highest
   * (always on)
   */
  public void analogWrite(int pin, int value) {
    pinMode(pin, PWM);
    out.write(ANALOG_MESSAGE | (pin & 0x0F));
    out.write(value & 0x7F);
    out.write(value >> 7);
  }

  /**
   * Write a value to a servo pin.
   *
   * @param pin the pin the servo is attached to
   * @param value the value: 0 being the lowest angle, and 180 the highest angle
   */
  public void servoWrite(int pin, int value) {
    out.write(ANALOG_MESSAGE | (pin & 0x0F));
    out.write(value & 0x7F);
    out.write(value >> 7);
  }
  /*
   * new bit of code
   * 
   * */
  /**
   * Configure a stepper motor using
   *
   * @param deviceNum the number identifying the specific stepper motor 0-9
   * @param stepPin pin connected to stepper motor step pin
   * @param dirPin pin connected to stepper motor direction pin
   */
  public void asConfig(int deviceNum, int stepPin, int dirPin) {
	    out.write(START_SYSEX);
	    out.write(ACCELSTEPPER_DATA);
	    out.write(ACCELSTEPPER_CONFIG);
	    out.write(deviceNum);
	    out.write(0100000); //2 wire configuration, full steps, no enable pin
	    out.write(stepPin);
	    out.write(dirPin);
	    out.write(END_SYSEX);
	    System.out.println("accelstepper config on pins " + stepPin + " & " + dirPin);
	  }
  /*
   * Set the zero position on a stepper motor
   * 
   *  @param deviceNum the number identifying the specific stepper motor 0-9
   * */
  
  public void asZero(int deviceNum) {
	    out.write(START_SYSEX);
	    out.write(ACCELSTEPPER_DATA);
	    out.write(ACCELSTEPPER_ZERO);
	    out.write(deviceNum);
	    out.write(END_SYSEX);
	    System.out.println("accelstepper zero on deviceNum " + deviceNum);
	  }
  public void asStep(int deviceNum, int steps) {
	  int[] arg= encode32bit(steps);
	  	out.write(START_SYSEX);
	    out.write(ACCELSTEPPER_DATA);
	    out.write(ACCELSTEPPER_STEP);
	    out.write(deviceNum);
	    out.write(arg[0]);  //samples from 0-6
	    out.write(arg[1]);  //samples from 7-13
	    out.write(arg[2]); //samples from 14-20
	    out.write(arg[3]); //samples from 21 to 27
	    out.write(arg[4]);	    
	    out.write(END_SYSEX);
	    System.out.println("accelstepper set to move"+steps+" steps on stepper " + deviceNum);
  }
  public void asTo(int deviceNum, int moveto) {
	  int[] arg= encode32bit(moveto);
	  	out.write(START_SYSEX);
	    out.write(ACCELSTEPPER_DATA);
	    out.write(ACCELSTEPPER_TO);
	    out.write(deviceNum);
	    out.write(arg[0]);  //samples from 0-6
	    out.write(arg[1]);  //samples from 7-13
	    out.write(arg[2]); //samples from 14-20
	    out.write(arg[3]); //samples from 21 to 27
	    out.write(arg[4]);	    
	    out.write(END_SYSEX);
	    System.out.println("accelstepper set to move to "+moveto+"absolute position on stepper " + deviceNum);
  }
  public void asStop(int deviceNum) {
	  	out.write(START_SYSEX);
	    out.write(ACCELSTEPPER_DATA);
	    out.write(ACCELSTEPPER_STOP);
	    out.write(deviceNum);	    
	    out.write(END_SYSEX);
	    System.out.println("stepper " + deviceNum + " was stopped!");
  }
  public void asReport(int deviceNum) {
	  	out.write(START_SYSEX);
	    out.write(ACCELSTEPPER_DATA);
	    out.write(ACCELSTEPPER_REPORT);
	    out.write(deviceNum);	    
	    out.write(END_SYSEX);
	    System.out.println("stepper " + deviceNum + " sent report command!");
}
  public void asSetSpeed(int deviceNum, float speed) {
	    int args = encodeCustomFloat(speed);
	    out.write(START_SYSEX);
	    out.write(ACCELSTEPPER_DATA);
	    out.write(ACCELSTEPPER_SETSPEED);
	    out.write(deviceNum);
	    out.write((args) & 127); //samples from 21 to 27
	    out.write((args>>7) & 127); //samples from 14-20
	    out.write((args>>14) & 127);  //samples from 7-13
	    out.write((args>>21) & 127);  //samples from 0-6
	    out.write(END_SYSEX);
	    System.out.println("accelstepper max speed set to "+speed+"steps/s on stepper " + deviceNum);
	  }
  public void asSetAccel(int deviceNum, float accel) {
	    int args = encodeCustomFloat(accel);
	    out.write(START_SYSEX);
	    out.write(ACCELSTEPPER_DATA);
	    out.write(ACCELSTEPPER_STEP);
	    out.write(deviceNum);
	    out.write((args) & 127); //samples from 21 to 27
	    out.write((args>>7) & 127); //samples from 14-20
	    out.write((args>>14) & 127);  //samples from 7-13
	    out.write((args>>21) & 127);  //samples from 0-6
	    out.write(END_SYSEX);
	    System.out.println("accelstepper accel set to "+accel+"steps/s^2 on stepper " + deviceNum);
	  }
  public void asMultiConfig(int groupNum, int deviceNum1, int deviceNum2) {
	    out.write(START_SYSEX);
	    out.write(ACCELSTEPPER_DATA);
	    out.write(ACCELSTEPPER_MULTICONF);
	    out.write(groupNum);
	    out.write(deviceNum1); //samples from 21 to 27
	    out.write(deviceNum2); //samples from 14-20
	    out.write(END_SYSEX);
	    System.out.println("multistepper group "+groupNum+"was setup with steppers" + deviceNum1 + " and " + deviceNum2);
	  }
  public void asMultiTo(int groupNum, int steps) {
	  int[] arg= encode32bit(steps);
	  out.write(START_SYSEX);
	    out.write(ACCELSTEPPER_DATA);
	    out.write(ACCELSTEPPER_MULTI_TO);
	    out.write(groupNum);
	    out.write(arg[0]);  //samples from 0-6
	    out.write(arg[1]);  //samples from 7-13
	    out.write(arg[2]); //samples from 14-20
	    out.write(arg[3]); //samples from 21 to 27
	    out.write(arg[4]);
	    out.write(END_SYSEX);
	    System.out.println("multistepper group "+groupNum+"was moved "+steps +"steps");
	  }
  public void asMultiStop(int groupNum) {
	    out.write(START_SYSEX);
	    out.write(ACCELSTEPPER_DATA);
	    out.write(ACCELSTEPPER_MULTI_STOP);
	    out.write(groupNum);
	    out.write(END_SYSEX);
	    System.out.println("multistepper group "+groupNum+"was stopped");
	  }

  
	public static int[] encode32bit(int val) {
		boolean inv=false;
		if(val<0) {inv=true;val=-val;}
		int[] args = new int[5];
		args[0] = val & 0x7F;
		args[1] = ((val >> 7) & 0x7F);
		args[2] = ((val >> 14) & 0x7F);
		args[3]= ((val >> 21) & 0x7F);
		args[4]= ((val >> 28) & 0x7F);
		if(inv) {args[4]=args[4] | 0x08;}
		return args;
	}
	public static int decode32bit(int arg0, int arg1, int arg2, int arg3, int arg4) {
		int value = arg0 | (arg1<<7) | (arg2<<14) | (arg3<<21) | ((arg4 << 28) & 0x07);
		if (arg4 >> 3 == 0x01) {
		    value = value*-1;
		  }
		return value;
	}
  
	public static int encodeCustomFloat(float decimal){    
		int exp = (int)(Math.log10(8388608/Math.abs(decimal))); //maximum precision of 23bit
		int placeholder=(int)(Math.abs(decimal)*Math.pow(10, exp));
		int power = 11-exp; //power is shifted down by 11 on the interpreter
		int sgn =0; int args=0;
		if (Math.signum(decimal)==-1) {
			sgn =1; args = (placeholder)|power<<23|sgn<<27;} 
		else {
			sgn =0; args = (placeholder)|power<<23;}
		//uses bit shifting to generate a 28bit number representing the custom float
		return args;
	}  
	public static float decodeCustomFloat(int encoded) {
		int significand = encoded & (1 << 23)-1;
//		System.out.println(significand);
		int exponent = (encoded >> 23) & 15;
//		System.out.println(exponent);
		int sign = (encoded >> 27 & 1); 
//		System.out.println(sign);
		float decimal = (float)significand*(float)Math.pow(10.0f, exponent - 11);
		if(sign == 1) {decimal=-1.0f*decimal;}
		return decimal;
	}
  private void setDigitalInputs(int portNumber, int portData) {
    //System.out.println("digital port " + portNumber + " is " + portData);
    digitalInputData[portNumber] = portData;
  }

  private void setAnalogInput(int pin, int value) {
    //System.out.println("analog pin " + pin + " is " + value);
    analogInputData[pin] = value;
  }

  private void setVersion(int majorVersion, int minorVersion) {
    //System.out.println("version is " + majorVersion + "." + minorVersion);
    this.majorVersion = majorVersion;
    this.minorVersion = minorVersion;
  }

  private void queryCapabilities() {
    out.write(START_SYSEX);
    out.write(CAPABILITY_QUERY);
    out.write(END_SYSEX);
  }

  private void queryAnalogMapping() {
    out.write(START_SYSEX);
    out.write(ANALOG_MAPPING_QUERY);
    out.write(END_SYSEX);
  }

  private void processSysexMessage() {
//    System.out.print("[ ");
//    for (int i = 0; i < storedInputData.length; i++) System.out.print(storedInputData[i] + " ");
//    System.out.println("]");
    switch(storedInputData[0]) { //first byte in buffer is command
//      case CAPABILITY_RESPONSE:
//        for (int pin = 0; pin < pinModes.length; pin++) {
//          pinModes[pin] = 0;
//        }
//        for (int i = 1, pin = 0; pin < pinModes.length; pin++) {
//          for (;;) {
//            int val = storedInputData[i++];
//            if (val == 127) break;
//            pinModes[pin] |= (1 << val);
//            i++; // skip mode resolution for now
//          }
//          if (i == sysexBytesRead) break;
//        }
//        for (int port = 0; port < pinModes.length; port++) {
//          boolean used = false;
//          for (int i = 0; i < 8; i++) {
//            if (pinModes[port * 8 + pin] & (1 << INPUT) != 0) used = true;
//          }
//          if (used) {
//            out.write(REPORT_DIGITAL | port);
//            out.write(1);
//          }
//        }
//        break;
      case ANALOG_MAPPING_RESPONSE:
        for (int pin = 0; pin < analogChannel.length; pin++)
          analogChannel[pin] = 127;
        for (int i = 1; i < sysexBytesRead; i++)
          analogChannel[i - 1] = storedInputData[i];
        for (int pin = 0; pin < analogChannel.length; pin++) {
          if (analogChannel[pin] != 127) {
            out.write(REPORT_ANALOG | analogChannel[pin]);
            out.write(1);
          }
        }
      case ACCELSTEPPER_DATA:
          switch(storedInputData[1]) {
          case ACCELSTEPPER_REPORT:
        	  for (int i = 2; i < sysexBytesRead; i++) {
        		  accelStepperChannel[i - 2] = storedInputData[i];}
        	  int repDeviceNum = accelStepperChannel[0];
        	  int repSteps = decode32bit(accelStepperChannel[1],accelStepperChannel[2],accelStepperChannel[3],accelStepperChannel[4],accelStepperChannel[5]);
        	 System.out.println("Stepper device "+ repDeviceNum + " is at position "+ repSteps);
        		  
          case ACCELSTEPPER_MOVECOMPLETE:
        	  for (int i = 2; i < sysexBytesRead; i++) {
        		  accelStepperChannel[i - 2] = storedInputData[i];}
        	  int moveDeviceNum = accelStepperChannel[0];
        	  int moveSteps = decode32bit(accelStepperChannel[1],accelStepperChannel[2],accelStepperChannel[3],accelStepperChannel[4],accelStepperChannel[5]);
        	  System.out.println("Stepper device "+ moveDeviceNum + " is at position "+ moveSteps);
        	  
          case ACCELSTEPPER_MULTI_MOVECOMPLETE:
        	  for (int i = 2; i < sysexBytesRead; i++) {
        		  accelStepperChannel[i - 2] = storedInputData[i];}
        	  int groupNum = accelStepperChannel[0];
        	  System.out.println("MultiStepper group "+ groupNum + " completed its steps");  
          } 
          break;
          }
//    	  for (int i = 1; i < sysexBytesRead; i++)
//            accelStepperChannel[i - 1] = storedInputData[i];
//         }
    	

    }


  public void processInput(int inputData) {
    int command;

//    System.out.print(">" + inputData + " ");

    if (parsingSysex) {
      if (inputData == END_SYSEX) {
        parsingSysex = false;
        processSysexMessage();
      } else {
        storedInputData[sysexBytesRead] = inputData;
        sysexBytesRead++;
      }
    } else if (waitForData > 0 && inputData < 128) {
      waitForData--;
      storedInputData[waitForData] = inputData;

      if (executeMultiByteCommand != 0 && waitForData == 0) {
        //we got everything
        switch(executeMultiByteCommand) {
        case DIGITAL_MESSAGE:
          setDigitalInputs(multiByteChannel, (storedInputData[0] << 7) + storedInputData[1]);
          break;
        case ANALOG_MESSAGE:
          setAnalogInput(multiByteChannel, (storedInputData[0] << 7) + storedInputData[1]);
          break;
        case REPORT_VERSION:
          setVersion(storedInputData[1], storedInputData[0]);
          break;
        }
      }
    } else {
      if(inputData < 0xF0) {
        command = inputData & 0xF0;
        multiByteChannel = inputData & 0x0F;
      } else {
        command = inputData;
        // commands in the 0xF* range don't use channel data
      }
      switch (command) {
      case DIGITAL_MESSAGE:
      case ANALOG_MESSAGE:
      case REPORT_VERSION:
        waitForData = 2;
        executeMultiByteCommand = command;
        break;
      case START_SYSEX:
        parsingSysex = true;
        sysexBytesRead = 0;
        break;
      }
    }
  }
}
