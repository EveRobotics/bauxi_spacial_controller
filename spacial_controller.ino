/*****************************************************************************/
/**
 * Spacial Navigation Controller implementation.
 *
 * Targeted for Sparkfun's Arduino Pro Mini 5V 16MHz microcontroller.
 *****************************************************************************/

#include <SPI.h>
#include <RFM69.h>

#include "moving_average.h"

//Includes required to use Roboclaw library
#include <SoftwareSerial.h>
#include "RoboClaw.h"

//#include <MemoryFree.h>

void(* resetFunc) (void) = 0; //declare reset function @ address 0



//==============================================================================
// PIN DECLARATIONS: TODO: fix these up for the pro mini.
//==============================================================================

enum ProcessorPins {
    // Mux 1 Pins, Inputs to processor from analog distance sensors:
    MUX_ANALOG_IN_S0 = 18,
    MUX_ANALOG_IN_S1 = 19,
    MUX_ANALOG_IN_S2 = 20,
    MUX_ANALOG_IN_S3 = 21,
    MUX_ANALOG_IN_SIG = A7, // Analog input for distance sensors and power monitoring.
    // Mux 2 Pins, Outputs from processor to chip select pins and UI elements:
    MUX_DIGITAL_OUT_S0 = 1,
    MUX_DIGITAL_OUT_S1 = 0,
    MUX_DIGITAL_OUT_S2 = 2,
    // int MUX2_S3 = NA, pull low with pull-down resistor to access first eight channels.
    MUX_DIGITAL_OUT_SIG = 3, // Digital output signal for chip selects and UI indicators.
    // Left motor (motor 1) speed controller outputs:
    MOTOR_PWM_LEFT = 9,
    MOTOR_CTRL1_LEFT = 4,
    MOTOR_CTRL2_LEFT = 5,
    // Right motor (motor 2) speed controller outputs:
    MOTOR_PWM_RIGHT = 10,
    MOTOR_CTRL1_RIGHT = 6,
    MOTOR_CTRL2_RIGHT = 7,
    // SPI bus I/O:
    SPI_MOSI = 16,
    SPI_MISO = 14,
    SPI_SERIAL_CLK = 15,
};

// Used for addressing the channels of the digital output multiplexer (mux2):
enum DigitalMuxChannels {
    C0_ENCODER_CNT_LEFT_CS  = 0, // Chip select for the first (left) encoder counter.
    C1_ENCODER_CNT_RIGHT_CS = 1, // Chip select for the right side encoder counter.
    // Sonar range-finder sonar transmit enable. Prevents interference...
    C2_SONAR_LEFT_ENABLE  = 2,
    C3_SONAR_FRONT_ENABLE = 3,
    C4_SONAR_RIGHT_ENABLE = 4,
    C5_SONAR_BACK_ENABLE  = 5,
    // Enumerate digital status outputs:
};

// Used for addressing the channels of the analog input multiplexer (mux1):
enum AnalogMuxChannels {
    C0_SONAR_LEFT  = 0, // Sonar left input signal. 
    C1_SONAR_FRONT = 1,
    C2_SONAR_RIGHT = 2,
    C3_SONAR_BACK  = 3,
    C4_IR_DIST_FRONT = 4,
    C5_IR_DIST_LEFT  = 5,
    C6_IR_DIST_RIGHT = 6,
    C7_IR_DIST_BACK  = 7,
    // Enumerate battery power monitoring channels:
    // Mux analog resolution may not be sufficient for current sensing:
    // i.e. voltage drop across a 0.1 Ohm resistor, if so, free 2 inputs
    // for user inputs (switches or potentiometers, or bump switches).
    // Enumerate User input controls:
    C8_RADIO_A = 8,
    C9_RADIO_B = 9,
    C10_RADIO_C = 10,
    C11_RADIO_D = 11,
};

///============================================================================
/// ANALOG MULTIPLEXER:
///============================================================================

/*****************************************************************************/
/**
 * Analog mux for reading sonar and IR and enabling the sonar transmit.
 *****************************************************************************/
class AnalogMultiplexer {

public:

    AnalogMultiplexer(int s0Pin, int s1Pin, int s2Pin, int s3Pin) {
        _s0Pin = s0Pin;
        _s1Pin = s1Pin;
        _s2Pin = s2Pin;
        _s3Pin = s3Pin;

        if(_s0Pin > -1) {
            pinMode(_s0Pin, OUTPUT);
        }
        if(_s1Pin > -1) {
            pinMode(_s1Pin, OUTPUT);
        }
        if(_s2Pin > -1) {
            pinMode(_s2Pin, OUTPUT);
        }
        if(_s3Pin > -1) {
            pinMode(_s3Pin, OUTPUT);
        }

        selectChannel(0);
    }
    

    void selectChannel(int channel) {
        _channel = channel;
        // Decode a nibble:
        int s0 = channel & 1;
        int s1 = (channel >> 1) & 1;
        int s2 = (channel >> 2) & 1;
        int s3 = (channel >> 3) & 1;

        //Serial.print("channel: ");
        //Serial.print(channel);
        //Serial.print("\n");
        
        if(_s0Pin > -1) {
            digitalWrite(_s0Pin, s0);
        }
        if(_s1Pin > -1) {
            digitalWrite(_s1Pin, s1);
        }
        if(_s2Pin > -1) {
            digitalWrite(_s2Pin, s2);
        }
        if(_s3Pin > -1) {
            digitalWrite(_s3Pin, s3);
        }
    }

private:

    int _s0Pin = 0;
    int _s1Pin = 0;
    int _s2Pin = 0;
    int _s3Pin = 0;

    int _channel = 0;
};


///============================================================================
/// RANGE FINDER (parent class):
///============================================================================

/*****************************************************************************/
/**
 * @brief Generic base class for rangefinder sensors.
******************************************************************************/
class Rangefinder {

public:

    virtual float readDistance(void);

    virtual float getDistance(void);

    virtual int getRawDistance(void);

    virtual ~Rangefinder(void) { };

protected:
    int _signalPin = 0;

    int _rawValue = 0;
    float _distance = 0;

    ExponentialMovingAverage* _average;

private:

};


///============================================================================
/// INFRARED RANGEFINDER:
///============================================================================

class InfraredRangefinder : public Rangefinder {

public:

    InfraredRangefinder(int signalPin) {
        _average = new ExponentialMovingAverage();
        _signalPin = signalPin;
    }

    float readDistance() {
        // Read the analog input, and calculate the distance.
        // TODO: implement function for calculating distance .
        return _distance;
    }

    float getDistance() {
        // Get the distance without performing a measurement.
        return _distance;
    }

    int getRawDistance() {
        // Get the distance without performing a measurement.
        return _rawValue;
    }


private:

};


///============================================================================
/// SONAR RANGEFINDER:
///============================================================================

/**************************************************************************/
/**
 * @brief LV-MaxSonar-EZTM Series Sonar Rangefinder class implementation.
 *
 * Common functionality may include input buffering and averaging
 *****************************************************************************/
class SonarRangefinder : public Rangefinder {

public:

    /**********************************************************************/
    /**
    * Initialize the sonar enable pin (Pin4 RX) and the analog signal output pin
    * (Pin3 AN).
    *
    * Measures 0 - 254 inches. (645.16 cm)
    * Speed of sound is 340.29 m / s and 34029 cm / s
    *
    * 34029 / 645.16 cm = 52.7450554901
    *
    * 1 s / 52.7450554901 = 0.0189591231 seconds
    * Double the time for the reflected signal.
    * 0.0189591231 * 2 = 0.0379182462 = 37.92 ms, 37918.2 us.
    **************************************************************************/
    SonarRangefinder(int enablePin, int signalPin) {
        _enablePin = enablePin;
        _signalPin = signalPin;
        // https://www.arduino.cc/en/Tutorial/DigitalPins
        // https://www.arduino.cc/en/Reference/PinMode
        pinMode(_enablePin, OUTPUT);
        // This pin is internally pulled high. The LV-MaxSonar-EZ will 
        // continually measure range and output if RX data is left unconnected 
        // or held high. If held low the sensor will stop ranging. Bring high 
        // for 20uS or more to command a range reading.
        digitalWrite(_enablePin, LOW);
        // We do not need to initialize the HW for the analog input pin.
        _average = new ExponentialMovingAverage();
    }

    float readDistance(void) {
        // Assert the enable pin for 20 microseconds or more, then perform a
        // reading on the signal pin. Return the reading
        digitalWrite(_enablePin, HIGH);
        delayMicroseconds(SAMPLE_DELAY);
        // Enable reading for enough time for at least a single sample at
        // maximum range.
        _rawValue = 0;
        for(int i = 0; i < SAMPLE_COUNT; i++) {
            delayMicroseconds(SAMPLE_DELAY);
            // https://www.arduino.cc/en/Tutorial/AnalogInput
            _rawValue += analogRead(_signalPin);
        }
        digitalWrite(_enablePin, LOW);
        delayMicroseconds(SAMPLE_DELAY);
        _rawValue = (float)_rawValue / (float)SAMPLE_COUNT;

        // Convert the raw value into distance (centimeters).
        _distance = (float)_rawValue / SCALE_FACTOR_CM;
        return _distance;
    }

    float getDistance() {
        // Get the distance without performing a measurement.
        return _distance;
    }

    int getRawDistance() {
        // Get the distance without performing a measurement.
        return _rawValue;
    }

private:

    int _enablePin = 0;

    const int SAMPLE_COUNT = 3;
    
    // Outputs analog voltage with a scaling factor of (Vcc / 512) per inch.
    // A supply of 5V yields ~9.8mV / inch and 3.3V yields ~6.4mV/inch.
    const float SCALE_FACTOR_IN = 1023 / 512;
    // 2.54 centimeters per inch.
    const float SCALE_FACTOR_CM = 1023 / 1300.48;
    // Delay enough time for sound to travel up to 645 cm to an object and 
    // be reflected back to the sensor.
    const int SAMPLE_DELAY = 37920; 
};

/*****************************************************************************/
/**
 * @brief Base class for receiving commands from a channel.
 *
 * @details Receive commands via a serial stream and change the operating modes
 *     and states of the system.
 *****************************************************************************/
class CommandInterpreter {

public:
    /*************************************************************************/
    /**
     * @brief Base class constructor
     *************************************************************************/
    CommandInterpreter(const String interpreterName): _name(interpreterName) {
        _runMode = 0;
        _speedLeft = 0;
        _speedRight = 0;
    }

    virtual ~CommandInterpreter(void) {};

    /**************************************************************************
     * @brief Read commands from a channel
     *
     * @details The implementation of the reader is interface specific.
     *************************************************************************/
    virtual void readCommands(void) {};

    void printState(void) {
        String s = "Cmd interpreter state: " + _name + "\n";
        s = s + "Left: " + getSpeedLeft() + "\n";
        s = s + "Right: " + getSpeedRight() + "\n";
        s = s + "Mode: " + getRunMode() + "\n";
        Serial.print(s);
    }

    /**************************************************************************
     * @brief Get the speed for the left motors.
     *************************************************************************/
    int getSpeedLeft() {
        return _speedLeft;
    }

    /**************************************************************************
     * @brief Get the speed for the right motors.
     *************************************************************************/
    int getSpeedRight() {
        return _speedRight;
    }

    /**************************************************************************
     * @brief Get the desired run mode.
     *
     * @returns AUTO_AVOID, EXT_CONTROL, or REMOTE_CONTROL
     *************************************************************************/
    int getRunMode() {
        return _runMode;
    }


protected:

    enum InputCommands {
        CMD_RESET       = 0x61, // 'a' abort, reset the micro-controller.
        CMD_SPEED_LEFT  = 0x6c, // 'l' left, valid speeds: (0, 255)
        CMD_SPEED_RIGHT = 0x72, // 'r' right, valid speeds: (0, 255)
        CMD_RUN_MODE    = 0x6d, // 'm' mode: 0 = AUTO_AVOID, 1 = EXT_CONTROL, 2 = REMOTE_CONTROL
        // TODO: command to read motor controller PID constants.
    };

    int _speedLeft;
    int _speedRight;
    int _runMode;

private:

    const String _name; // The name of the controller, used for debug printing.
};

/******************************************************************************
 * @brief Radio module command interpreter.
 *
 * @details
 *****************************************************************************/
class CommandInterpreterRadio : public CommandInterpreter {

public:
    CommandInterpreterRadio(void) : CommandInterpreter(String("Radio")) {
        // Initialize the RFM69HCW:
        if(_radio.initialize(FREQUENCY, MYNODEID, NETWORKID)) {
            Serial.print("Radio init success\n");
        } else {
            Serial.print("Radio init failure\n");
        }
        _radio.setHighPower(); // Always use this for RFM69HCW
        _radio.setCS(10);
        // Turn on encryption if desired:
        if (ENCRYPT) {
            _radio.encrypt(ENCRYPTKEY);
        }
    }

    void readCommands() {
        if(_radio.receiveDone()) {
            _noDataCounter = 0;
            bool commandValid = true;
            for (byte i = 1; i < _radio.DATALEN; i++) {
                if((char)_radio.DATA[i] == ':' && _radio.DATALEN > i) {
                    unsigned char command = (char)_radio.DATA[i-1];
                    if(command == CMD_SPEED_LEFT) {
                        _speedLeft = (int)_radio.DATA[i+1];
                    } else if(command == CMD_SPEED_RIGHT) {
                        _speedRight = (int)_radio.DATA[i+1];
                    } else if(command == CMD_RUN_MODE) {
                        _runMode = (int)_radio.DATA[i+1];
                    } else if(command == CMD_RESET) {
                        resetFunc();  //call reset
                    } else {
                        commandValid = false;
                    }
                }
            }
            // Save the signal strength, maybe it will be interesting to see.
            _receiveSignalStrength = _radio.RSSI;
            if((char)_radio.DATA[_radio.DATALEN-1] != ';' || !commandValid) {
                Serial.print("RX invalid cmd or EOM, radio.\n");
                Serial.print("Data: ");
                for (byte i = 0; i < _radio.DATALEN; i++) {
                    Serial.print((char)_radio.DATA[i]);
                }
                Serial.print("\nRSSI: ");
                Serial.print(_receiveSignalStrength);
                Serial.print("\n");
            }
        } else {
            // TODO: if we don't receive packets in a while shut down motion.
            _noDataCounter++;
            if(_noDataCounter > NO_DATA_COUNT_LIMIT) {
                _speedLeft = 128;
                _speedRight = 128;
                _runMode = MODE_MOTION_DISABLED;
                Serial.print("No radio!\n");
            }
        }
    }

    short getReceiveSignalStrength(void) {
        return _receiveSignalStrength;
    }

    /*************************************************************************/
    /**
     * @brief Get a reference to the radio module
     *
     * @details Do this so we can share the radio with a telemetry system in
     *     the future.
     *************************************************************************/
    RFM69* getRadioReference(void) {
        return &_radio;
    }

    enum RunModes {
        MODE_MOTION_DISABLED = 0,
        MODE_MOTION_ENABLED_REMOTE = 1,
        MODE_MOTION_ENABLED_AUTO = 2,
    };

private:
    const unsigned char NETWORKID = 100; // Must be the same for all nodes (0 to 255)
    const unsigned char MYNODEID  = 4;   // My node ID (0 to 255)
    const unsigned char TONODEID  = 255; // Destination node ID (0 to 254, 255 = broadcast)
    const unsigned char FREQUENCY = RF69_915MHZ;
    // AES encryption (or not):
    const bool ENCRYPT     = true;               // Set to "true" to use encryption
    const char* ENCRYPTKEY = "sampleEncryptKey"; // Use the same 16-byte key on all nodes
    // Use ACKnowledge when sending messages (or not):
    const bool USEACK = true; // Request ACKs or not

    const int NO_DATA_COUNT_LIMIT = 10;
    RFM69 _radio;
    short _receiveSignalStrength = 0;
    int _noDataCounter = 0;
};

/*****************************************************************************/
/**
 * @brief Serial interface based command interpreter.
 *
 * @details Process received serial data for configuration and control commands.
 *****************************************************************************/
class CommandInterpreterSystem : public CommandInterpreter {

public:
    CommandInterpreterSystem(void) : CommandInterpreter("System") { }

    void readCommands() {
        bool commandValid = true;
        while(Serial.available() > 0) {
            // Read the incoming bytes:
            unsigned char command = Serial.read();
            Serial.print("Received command: ");
            Serial.print(command);
            Serial.print(", param: ");
            if(Serial.read() == ':') {
                if(command == CMD_SPEED_LEFT) {
                    _speedLeft = Serial.parseInt();
                    Serial.print(_speedLeft);
                } else if(command == CMD_SPEED_RIGHT) {
                    _speedRight = Serial.parseInt();
                    Serial.print(_speedRight);
                } else if(command == CMD_RUN_MODE) {
                    _runMode = Serial.parseInt();
                    Serial.print(_runMode);
                } else if(command == CMD_RESET) {
                    Serial.print("\nResetting controller.\n");
                    Serial.flush();
                    resetFunc();  //call reset
                } else {
                    commandValid = false;
                }
                Serial.print("\n");
                // Parse user indicator commands and configuration commands.
                if(Serial.read() != ';' && !commandValid) {
                    Serial.print("RX invalid cmd or EOM, system.\n");
                }
            }
        }
    }

    enum RunModes {
        MODE_AUTO_AVOID = 0,
        MODE_SYS_CONTROL = 1,
        MODE_REMOTE_CONTROL = 2,
    };

private:

};

/*****************************************************************************/
/**
 * @brief Simple wrapper class to execute roboclaw functions and capture state.
 *
 * @details Store status of function execution, motor tuning parameters, etc..
 *****************************************************************************/
class MotorController {

public:

    MotorController(RoboClaw* claw) {
        _claw = claw;
        // Set the encoder counts to half of uint32_t.max
        if(_claw->SetEncM1(CTL_ADDRESS, 0x7fffffff) == FAILURE) {
            Serial.print("Set M1 encoder failed\n");
        }
        if(_claw->SetEncM2(CTL_ADDRESS, 0x7fffffff) == FAILURE) {
            Serial.print("Set M2 encoder failed\n");
        }
        //Set PID Coefficients, Kp, Ki, Kd,
        if(_claw->SetM1VelocityPID(CTL_ADDRESS, KP,KI,KD, QPPS) == FAILURE) {
            Serial.print("Setup M1 PID failed\n");
        }
        if(_claw->SetM2VelocityPID(CTL_ADDRESS, KP,KI,KD, QPPS) == FAILURE) {
            Serial.print("Setup M2 PID failed\n");
        }
        // Set Acceleration:
        if(_claw->SetM1DefaultAccel(CTL_ADDRESS, ACCELERATION) == FAILURE) {
            Serial.print("Set M1 accel failed\n");
        }
        if(_claw->SetM2DefaultAccel(CTL_ADDRESS, ACCELERATION) == FAILURE) {
            Serial.print("Set M2 accel failed\n");
        }
    }

    void readEncoderCounts(void) {

        bool valid = false;
        _encCountsLeft = _claw->ReadEncM2(CTL_ADDRESS, &_encStatusM1, &valid);
        if(valid == INVALID) {
            Serial.print("Read M1 enc, data invalid\n");
        }
        _encCountsRight = _claw->ReadEncM1(CTL_ADDRESS, &_encStatusM2, &valid);
        if(valid == INVALID) {
            Serial.print("Read M2 enc, data invalid\n");
        }
    }

    unsigned long getEncoderCountsLeft(void) {
        return _encCountsLeft;
    }

    unsigned long getEncoderCountsRight(void) {
        return _encCountsRight;
    }

    void setSpeed(int speedLeft, int speedRight) {
        _claw->SpeedM2(CTL_ADDRESS, speedLeft);
        _claw->SpeedM1(CTL_ADDRESS, speedRight);
    }

private:

    RoboClaw* _claw;

    unsigned long _encCountsLeft = 0;
    unsigned long _encCountsRight = 0;
    unsigned char _encStatusM1 = 0;
    unsigned char _encStatusM2 = 0;

    const unsigned char CTL_ADDRESS = 0x80;
    //Velocity PID coefficients
    const float KP = 1.0;
    const float KI = 0.5;
    const float KD = 0.25;
    const unsigned int QPPS = 1300;
    const unsigned int ACCELERATION = 5000;
    const bool INVALID = false;
    const bool FAILURE = false;

};

/*****************************************************************************/
/**
 * @brief Platform controller class implementation.
 *
 * @details Centralized access to the hardware connected to the controller.
 *****************************************************************************/
class PlatformController {

public:
    PlatformController(RoboClaw* motorController) {

        _microseconds = 0;
        _milliseconds = 0;
        _prevMilliseconds = 0;
        _prevMicroseconds = 0;
        _prevEncoderCountsLeft = 0;
        _prevEncoderCountsRight = 0;

        _sonarLeft = new SonarRangefinder(MUX_DIGITAL_OUT_SIG, MUX_ANALOG_IN_SIG);
        _sonarFront = new SonarRangefinder(MUX_DIGITAL_OUT_SIG, MUX_ANALOG_IN_SIG);
        _sonarRight = new SonarRangefinder(MUX_DIGITAL_OUT_SIG, MUX_ANALOG_IN_SIG);
        _sonarBack = new SonarRangefinder(MUX_DIGITAL_OUT_SIG, MUX_ANALOG_IN_SIG);

        _infraredLeft = new InfraredRangefinder(MUX_ANALOG_IN_SIG);
        _infraredRight = new InfraredRangefinder(MUX_ANALOG_IN_SIG);
        _infraredBack = new InfraredRangefinder(MUX_ANALOG_IN_SIG);

        _muxAnalogInput = new AnalogMultiplexer(MUX_ANALOG_IN_S0, MUX_ANALOG_IN_S1
                , MUX_ANALOG_IN_S2, MUX_ANALOG_IN_S3);

        _muxDigitalOutput = new AnalogMultiplexer(MUX_DIGITAL_OUT_S0
                , MUX_DIGITAL_OUT_S1, MUX_DIGITAL_OUT_S2, -1);

        _motorController = new MotorController(motorController);

        _commandsSystem = new CommandInterpreterSystem();
        _commandsRadio = new CommandInterpreterRadio();
    }

    void pollSensorData(void) {
        // Poll the sonar, IR and other sensors, bumper etc..
        _readSonarLeft(); // Possibly combine the acquisition step.
        _readSonarFront();
        _readSonarRight();
        _readSonarBack();
        // Next get the IR sensors:
        _readInfraredLeft();
        _readInfraredRight();
        _readInfraredBack();
        // Next get bumper switch states:
        _readBumperSwitchStates();
        // Next get the motor encoder counts and other motor related stuff.
        _readEncoderCounts();
    }

    ///----------------------------------------------------
    /// @brief Rangefinder accessors. Side effect free.
    ///----------------------------------------------------
    float getSonarLeft(void) {
        return _sonarDistanceLeft;
    }

    float getSonarFront(void) {
        // Return the front sonar distance in centimeters.
        return _sonarDistanceFront;
    }

    float getSonarRight(void) {
        return _sonarDistanceRight;
    }

    float getSonarBack(void) {
        return _sonarDistanceBack;
    }

    ///----------------------------------------------------
    /// Infrared range-finders, distance in centimeters.
    ///----------------------------------------------------
    float getInfraredLeft(void) {
        return _infraredDistanceLeft;
    }

    float getInfraredRight(void) {
        return _infraredDistanceRight;
    }

    float getInfraredBack(void) {
        return _infraredDistanceBack;
    }

    ///----------------------------------------------------
    /// Bumper switch states, boolean: SWITCH_PRESSED or SWITCH_NOT_PRESSED
    ///----------------------------------------------------
    bool getBumperSwitchStateLeft(void) {
        return _bumperStateLeft;
    }

    bool getBumperSwitchStateRight(void) {
        return _bumperStateRight;
    }

    ///----------------------------------------------------
    /// Motor encoder counts.
    ///----------------------------------------------------

    unsigned long getEncoderCounterLeft() {
        return _motorController->getEncoderCountsLeft();
    }

    unsigned long getEncoderCounterRight() {
        return _motorController->getEncoderCountsRight();
    }

    void setTimestamp(unsigned long milliseconds, unsigned int microseconds) {
        _prevMilliseconds = _milliseconds; // Store the previous time-stamp.
        _prevMicroseconds = _microseconds;
        _milliseconds = milliseconds; // Store the current time-stamp.
        _microseconds = microseconds;
        // Now we have delta T in microseconds.
        _deltaTime = ((_milliseconds * 1000) + _microseconds)
            - ((_prevMilliseconds * 1000) + _prevMicroseconds);
    }

    void setSpeed(int speedLeft, int speedRight) {
        _motorController->setSpeed(speedLeft, speedRight);
    }

    unsigned long getMotorStatusLeft(void) {
        return 0;
    }

    unsigned long getMotorStatusRight(void) {
        return 0;
    }

    /*************************************************************************/
    /**
     * @brief Process commands from the system controller and radio controller.
     *
     * @details Check the mode values from the system and remote control
     *     channels. Take action based on the mode settings and state input.
     *************************************************************************/
    bool processControlInput(void) {
        _commandsSystem->readCommands();
        _commandsRadio->readCommands();

        _commandsSystem->printState();
        _commandsRadio->printState();

        bool motionEnable = (_commandsRadio->getRunMode()
                == _commandsRadio->MODE_MOTION_ENABLED_AUTO)
                && (_commandsRadio->getRunMode()
                    == _commandsRadio->MODE_MOTION_ENABLED_REMOTE);

        if(motionEnable) {
            unsigned char speedLeft = 0;
            unsigned char speedRight = 0;
            if((_commandsSystem->getRunMode()
                    == _commandsSystem->MODE_SYS_CONTROL)
                    && (_commandsRadio->getRunMode()
                            == _commandsRadio->MODE_MOTION_ENABLED_AUTO)) {

                speedLeft = _commandsSystem->getSpeedLeft();
                speedRight = _commandsSystem->getSpeedRight();
            } else if(_commandsRadio->getRunMode()
                    == _commandsRadio->MODE_MOTION_ENABLED_REMOTE) {

                speedLeft = _commandsRadio->getSpeedLeft();
                speedRight = _commandsRadio->getSpeedRight();
            }
            setSpeed(speedLeft, speedRight);
        }
        return motionEnable && _commandsSystem->getRunMode()
                == _commandsSystem->MODE_AUTO_AVOID;
    }

    void printLoopTiming(void) {
        Serial.print("Loop timing: ");
        Serial.print(_deltaTime / 1000.0);
        Serial.print(" mS\n");
    }

private:

    void _readSonarLeft(void) {
        // Ping the front sonar and get the distance in centimeters.
        _setSonarMux(C2_SONAR_LEFT_ENABLE, C0_SONAR_LEFT);
        _sonarDistanceLeft = _sonarLeft->readDistance();
    }

    void _readSonarFront(void) {
        _setSonarMux(C3_SONAR_FRONT_ENABLE, C1_SONAR_FRONT);
        _sonarDistanceFront = _sonarFront->readDistance();
    }

    void _readSonarRight(void) {
        _setSonarMux(C4_SONAR_RIGHT_ENABLE, C2_SONAR_RIGHT);
        _sonarDistanceRight = _sonarRight->readDistance();
    }

    void _readSonarBack(void) {
        _setSonarMux(C5_SONAR_BACK_ENABLE, C3_SONAR_BACK);
        _sonarDistanceBack = _sonarBack->readDistance();
    }

    ///----------------------------------------------------
    /// Infrared range-finder data acquisition.
    ///----------------------------------------------------

    void _readInfraredLeft(void) {
        _muxAnalogInput->selectChannel(C5_IR_DIST_LEFT);
        _infraredDistanceLeft = _infraredLeft->readDistance();
    }

    void _readInfraredRight(void) {
        _muxAnalogInput->selectChannel(C6_IR_DIST_RIGHT);
        _infraredDistanceRight = _infraredRight->readDistance();
    }

    void _readInfraredBack(void) {
        _muxAnalogInput->selectChannel(C7_IR_DIST_BACK);
        _infraredDistanceBack = _infraredBack->readDistance();
    }

    void _readBumperSwitchStates(void) {
        _muxAnalogInput->selectChannel(C7_IR_DIST_BACK);
        _bumperStateLeft = _infraredBack->readDistance();
        _muxAnalogInput->selectChannel(C7_IR_DIST_BACK);
        _bumperStateRight = _infraredBack->readDistance();
    }

    void _readEncoderCounts(void) {
        _prevEncoderCountsLeft = _motorController->getEncoderCountsLeft();
        _prevEncoderCountsRight = _motorController->getEncoderCountsRight();
        // Read the encoder counts from the motor controller.
        _motorController->readEncoderCounts();
    }

    void _checkCommandsSystem(void) {
        _commandsSystem->readCommands();
    }

    void _checkCommandsRadio(void) {
        _commandsRadio->readCommands();
    }

    void _setSonarMux(int enablePin, int signalPin) {
        _muxDigitalOutput->selectChannel(enablePin);
        _muxAnalogInput->selectChannel(signalPin);
    }

    SonarRangefinder* _sonarFront;
    SonarRangefinder* _sonarLeft;
    SonarRangefinder* _sonarRight;
    SonarRangefinder* _sonarBack;

    InfraredRangefinder* _infraredLeft;
    InfraredRangefinder* _infraredRight;
    InfraredRangefinder* _infraredBack;

    float _sonarDistanceLeft = 0.0;
    float _sonarDistanceFront = 0.0;
    float _sonarDistanceRight = 0.0;
    float _sonarDistanceBack = 0.0;

    float _infraredDistanceLeft = 0.0;
    float _infraredDistanceRight = 0.0;
    float _infraredDistanceBack = 0.0;

    // Multiplexer for analog input signals such as distance sensors, etc..
    AnalogMultiplexer *_muxAnalogInput;
    // Multiplexer for analog outputs, but we are only using it for digital.
    AnalogMultiplexer *_muxDigitalOutput;

    MotorController *_motorController; // Wraps the RoboClaw.

    unsigned long _milliseconds;
    unsigned int _microseconds;

    unsigned long _prevMilliseconds;
    unsigned int _prevMicroseconds;

    unsigned long _deltaTime = 0; // Time in microseconds between now an previous.

    unsigned long _prevEncoderCountsLeft;
    unsigned long _prevEncoderCountsRight;

    CommandInterpreterSystem *_commandsSystem;
    CommandInterpreterRadio *_commandsRadio;

    bool _bumperStateLeft = false;
    bool _bumperStateRight = false;
};

// Format sensor data and other platform data into messages to send to the PC:
class MessageFormatter {

public:
    MessageFormatter() {
        _message = String("s:");
    }

    void sendMessage(PlatformController *ctrl, unsigned long milliseconds
            , unsigned int microseconds) {

        _message = "s:";
        _message = _message + ctrl->getSonarLeft() + ",";
        _message = _message + ctrl->getSonarFront() + ",";
        _message = _message + ctrl->getSonarRight() + ",";
        _message = _message + ctrl->getSonarBack() + ",";
        _message = _message + 0 + ",";
        _message = _message + ctrl->getEncoderCounterLeft() + ",";
        _message = _message + ctrl->getEncoderCounterRight() + ",";
        // Motor status is: byte 2: speed (process variable), byte 1: set-point
        // byte 0: control variable. if motor enable is true, low bit of byte 3 is set.
        _message = _message + (unsigned long)0 + ",";
        // Same as above, but operating mode is in high byte.
        _message = _message + (unsigned long)0 + ",";
        _message = _message + (int)0 + ",";
        _message = _message + (int)0 + ",";
        _message = _message + (int)0 + ",";
        _message = _message + (int)0 + ",";
        _message = _message + (unsigned long)milliseconds + ",";
        _message = _message + (unsigned long)microseconds + ",";
        Serial.print(_message + "\n");
    }
    
private:
    String _message;
};

const float DIST_THRESHOLD_MAX = 100.0;
const float DIST_THRESHOLD_MED = 25.0;
const float DIST_THRESHOLD_MIN = 15.0;

const unsigned char SPEED_STOP   = 128;
const unsigned char SPEED_CRAWL  = 140;
const unsigned char SPEED_SLOW   = 150;
const unsigned char SPEED_MEDIUM = 160;
const unsigned char SPEED_FAST   = 200;

static unsigned long moveUntil = 0;
static unsigned char autoSpeedLeft = 0;
static unsigned char autoSpeedRight = 0;

/*****************************************************************************/
/**
 * @brief Drive around and avoid obstacles using range-finder readings.
 *
 * @details Take commands from the system controller and be sensitive to the
 *     dead-man switch.
 *****************************************************************************/
void processAutoAvoidance(PlatformController *ctrl) {
    Serial.print("Auto avoiding\n");
    float sonarLeftDist = ctrl->getSonarLeft();
    float sonarFrontDist = ctrl->getSonarFront();
    float sonarRightDist = ctrl->getSonarRight();
    float sonarBackDist = ctrl->getSonarBack();
    
    // And not within absolute minimum thresholds. 
    if(moveUntil > millis() 
        && ((sonarFrontDist > DIST_THRESHOLD_MIN
            && sonarLeftDist > DIST_THRESHOLD_MIN 
            && sonarRightDist > DIST_THRESHOLD_MIN
            && sonarBackDist > DIST_THRESHOLD_MIN)
                || (autoSpeedLeft == 0 && autoSpeedRight == 0))) {
        // We are executing some kind of evasive maneuver.
        // Move until the time is up.
        ctrl->setSpeed(autoSpeedLeft, autoSpeedRight);
        return;
    }
    
    // Idea: scale the left and right sonar ranges to the left and right 
    // motor outputs. Use a threshold distance for all range-finder types to
    // determine whether we move forward or backward.

    // Say the farthest sonar distance is 100 cm, then map distances from zero
    // to 100 to 0 to 10.

    // TODO: scale the proportional turn rate by the proximity of the front 
    // sonar sensor to an object. 
    if(sonarFrontDist > DIST_THRESHOLD_MAX
        && sonarLeftDist > DIST_THRESHOLD_MAX
        && sonarRightDist > DIST_THRESHOLD_MAX) {
        // Go fast!
        //Serial.print("Case 1: L=SPEED_FAST, R=SPEED_FAST\n");
        autoSpeedLeft = SPEED_FAST;
        autoSpeedRight = SPEED_FAST;
    } else if((sonarFrontDist <= DIST_THRESHOLD_MAX
        || sonarLeftDist <= DIST_THRESHOLD_MAX
        || sonarRightDist <= DIST_THRESHOLD_MAX)
            && (sonarBackDist > DIST_THRESHOLD_MIN
                || (sonarLeftDist > DIST_THRESHOLD_MED
                    || sonarLeftDist > DIST_THRESHOLD_MED
                    || sonarFrontDist > DIST_THRESHOLD_MED))) {
        // 3 cases, all three range-finders > medium distance.
        // all three range-finders greater than minimum distance
        // 1 of three range-finders less than minimum distance.
        if(sonarFrontDist > DIST_THRESHOLD_MED
            && sonarLeftDist > DIST_THRESHOLD_MED
            && sonarRightDist > DIST_THRESHOLD_MED) {
            // Maintain forward travel but steer proportionally.
            autoSpeedLeft = sonarRightDist / 10;
            autoSpeedRight = sonarLeftDist / 10;
        } else if(sonarFrontDist > DIST_THRESHOLD_MIN
            && sonarLeftDist > DIST_THRESHOLD_MIN
            && sonarRightDist > DIST_THRESHOLD_MIN) {
            
            if(sonarFrontDist > DIST_THRESHOLD_MED 
                && (sonarLeftDist > DIST_THRESHOLD_MED
                    || sonarRightDist > DIST_THRESHOLD_MED)) {
                // Stuff is not too far away, and we can proportionally turn.
                // Maybe go a bit slower.
                autoSpeedLeft = sonarRightDist / 10;
                autoSpeedRight = sonarLeftDist / 10;
            } else {
                moveUntil = millis() + 1500;
                // Use zero turning radius travel turn away from nearest thing.
                if(sonarLeftDist < sonarRightDist) {
                    autoSpeedLeft = SPEED_FAST;
                    autoSpeedRight = SPEED_STOP - (SPEED_MEDIUM - SPEED_STOP);
                } else {
                    autoSpeedLeft = SPEED_STOP - (SPEED_MEDIUM - SPEED_STOP);
                    autoSpeedRight = SPEED_FAST;
                }
            }
        } else {
            moveUntil = millis() + 2000;
            // Go in reverse, turn away from nearest thing.
            if(sonarBackDist > DIST_THRESHOLD_MED) {
                if(sonarLeftDist < sonarRightDist) {
                    autoSpeedLeft = SPEED_STOP - (SPEED_CRAWL - SPEED_STOP);
                    autoSpeedRight = SPEED_STOP - (SPEED_FAST - SPEED_STOP);
                } else {
                    autoSpeedLeft = SPEED_STOP - (SPEED_FAST - SPEED_STOP);
                    autoSpeedRight = SPEED_STOP - (SPEED_CRAWL - SPEED_STOP);
                }
            } else {
                // Zero turning radius turn.
                if(sonarLeftDist < sonarRightDist) {
                    autoSpeedLeft = SPEED_MEDIUM;
                    autoSpeedRight = SPEED_STOP - (SPEED_MEDIUM - SPEED_STOP);
                } else {
                    autoSpeedLeft = SPEED_STOP - (SPEED_MEDIUM - SPEED_STOP);
                    autoSpeedRight = SPEED_MEDIUM;
                }
            }
        }
    } else {
        moveUntil = millis() + 500;
        // Stop we can't go anywhere.
        autoSpeedLeft = 0;
        autoSpeedRight = 0;
    }

    ctrl->setSpeed(autoSpeedLeft, autoSpeedRight);
    return;
}

// Using SoftwareSerial. See limitations of Arduino SoftwareSerial
SoftwareSerial serial(10, 11);
RoboClaw roboclaw(&serial, 10000);

// Create global reference to the platform controller to use
// in the main loop for reading sensor data and performing motor control.
PlatformController controller(&roboclaw);

// Initialize sensors and platform controller:
void setup() {
    Serial.begin(57600);
    serial.begin(57600);
    randomSeed(analogRead(8));
    Serial.print("freeMemory()=");
    //Serial.print(freeMemory());
    Serial.print("\n");
}

unsigned int microseconds = 0;
unsigned long milliseconds = 0;

MessageFormatter messageFormatter;

/*****************************************************************************/
/**
 * @brief Arduino loop function.
 *
 * 1. Poll range finders and other data inputs
 * 2. Print data
 * 3. Process control input
 * 4. Take actions
 *****************************************************************************/
void loop() {
    controller.pollSensorData();
    // Get the time since the program started:
    milliseconds = millis();
    microseconds = micros() % 1000;
    messageFormatter.sendMessage(&controller, milliseconds, microseconds);
    if(controller.processControlInput()) {
        // Auto avoid is selected.
        processAutoAvoidance(&controller);
    }
    controller.setTimestamp(milliseconds, microseconds);
    return;
}

