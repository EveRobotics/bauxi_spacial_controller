/***************************************************************************/
/**
 * Spacial Navigation Controller implementation.
 *
 * Targeted for Sparkfun's Arduino Pro Mini 5V 16MHz microcontroller.
 *
Sketch uses 19,260 bytes (62%) of program storage space. Maximum is 30,720 bytes.
Global variables use 615 bytes (30%) of dynamic memory, leaving 1,433 bytes for local variables. Maximum is 2,048 bytes.
 *****************************************************************************/

#include <SPI.h>
#include <RFM69.h>

#define USE_RANGEFINDER_AVERAGING 1

#if USE_RANGEFINDER_AVERAGING
#include "moving_average.h"
#endif

//Includes required to use Roboclaw library
#include <SoftwareSerial.h>
#include "RoboClaw.h"

#define SHOW_FREE_MEMORY 1
#define DEBUG_PRINT 0

#if SHOW_FREE_MEMORY
#include <MemoryFree.h>
#endif

void(*resetFunc)(void) = 0; //declare reset function @ address 0

const unsigned char SPEED_STOP = 128;

//==============================================================================
// PIN DECLARATIONS:
//==============================================================================

enum ProcessorPins {
    // Mux 1 Pins, Inputs to processor from analog distance sensors:
    MUX_ANALOG_IN_S0 = A3,
    MUX_ANALOG_IN_S1 = A2,
    MUX_ANALOG_IN_S2 = A1,
    MUX_ANALOG_IN_S3 = 5,
    MUX_ANALOG_IN_SIG = A0, // Analog input for distance sensors and power monitoring.
    // Mux 2 Pins, Outputs from processor to chip select pins and UI elements:
    MUX_DIGITAL_OUT_S0 = 9,
    MUX_DIGITAL_OUT_S1 = 8,
    MUX_DIGITAL_OUT_S2 = 7,
    // Pull S3 down with a 10K
    MUX_DIGITAL_OUT_SIG = 6, // Digital output signal for chip selects and UI indicators.
    // SPI bus I/O:
    SPI_CS   = 10,
    SPI_MOSI = 11,
    SPI_MISO = 12,
    SPI_SCLK = 13,
    RFM69_RST = A4,
    RFM69_IRQ = 2,
    // Motor enable pin, assert this to switch power to the motor controller
    MOTOR_ENABLE = A5,
    // Pin 3 and 4 for Software Serial RX and TX for motor controller comms.
    MOTOR_RX = 3,
    MOTOR_TX = 4,
};

// Used for addressing the channels of the digital output multiplexer (mux2):
enum DigitalMuxChannels {
    // Sonar range-finder sonar transmit enable. Prevents interference...
    C0_SONAR_ENABLE_LEFT  = 0,
    C1_SONAR_ENABLE_FRONT = 1,
    C2_SONAR_ENABLE_RIGHT = 2,
    C3_SONAR_ENABLE_BACK  = 3,
    // Enumerate digital status outputs:
};

// Used for addressing the channels of the analog input multiplexer (mux1):
enum AnalogMuxChannels {
    C0_SONAR_LEFT  = 0, // Sonar left input signal. 
    C1_SONAR_FRONT = 1,
    C2_SONAR_RIGHT = 2,
    C3_SONAR_BACK  = 3,
    C4_IR_DIST_LEFT = 4,
    C5_IR_DIST_RIGHT  = 5,
    C6_IR_DIST_BACK = 6,
    C7_BUMPER_LEFT  = 7,
    C8_BUMPER_RIGHT  = 8,
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

    AnalogMultiplexer(unsigned char s0Pin, unsigned char s1Pin
            , unsigned char s2Pin, unsigned char s3Pin) {

        _s0Pin = s0Pin;
        _s1Pin = s1Pin;
        _s2Pin = s2Pin;
        _s3Pin = s3Pin;

        if(_s0Pin != UNUSED) {
            pinMode(_s0Pin, OUTPUT);
        }
        if(_s1Pin != UNUSED) {
            pinMode(_s1Pin, OUTPUT);
        }
        if(_s2Pin != UNUSED) {
            pinMode(_s2Pin, OUTPUT);
        }
        if(_s3Pin != UNUSED) {
            pinMode(_s3Pin, OUTPUT);
        }

        selectChannel(0);
    }
    

    void selectChannel(unsigned char channel) {
        _channel = channel;
        // Decode a nibble:
        unsigned char s0 = channel & 1;
        unsigned char s1 = (channel >> 1) & 1;
        unsigned char s2 = (channel >> 2) & 1;
        unsigned char s3 = (channel >> 3) & 1;
        
        if(_s0Pin != UNUSED) {
            digitalWrite(_s0Pin, s0);
        }
        if(_s1Pin != UNUSED) {
            digitalWrite(_s1Pin, s1);
        }
        if(_s2Pin!= UNUSED) {
            digitalWrite(_s2Pin, s2);
        }
        if(_s3Pin != UNUSED) {
            digitalWrite(_s3Pin, s3);
        }
        delayMicroseconds(6);
    }

    const unsigned char UNUSED = 255;

private:

    unsigned char _s0Pin = 0;
    unsigned char _s1Pin = 0;
    unsigned char _s2Pin = 0;
    unsigned char _s3Pin = 0;

    unsigned char _channel = 0;
};


///============================================================================
/// RANGE FINDER (parent class):
///============================================================================

/*****************************************************************************/
/**
 * @brief Generic base class for range-finder sensors.
******************************************************************************/
class Rangefinder {

public:

    virtual void readDistance(void);

    unsigned short getDistance(void) {
        // Get the distance without performing a measurement, not-averaged.
        return _distance;
    }

    unsigned short getDistanceAverage(void) {
        // Get the distance without performing a measurement.

#if USE_RANGEFINDER_AVERAGING
        return _average->computeAverage();
#else
        return _distance;
#endif

    }

    int getRawDistance(void) {
        // Get the distance without performing a measurement.
        return _rawValue;
    }

    virtual ~Rangefinder(void) { };

protected:
    unsigned char _signalPin = 0;

    int _rawValue = 0;
    unsigned short _prevRawValue = 0;
    unsigned short _distance = 0;

#if USE_RANGEFINDER_AVERAGING
    ExponentialMovingAverage* _average;
#endif

private:

};


///============================================================================
/// INFRARED RANGEFINDER:
///============================================================================

/**************************************************************************/
/**
 * For fast calculation of distance, given the output voltage of the sensor,
 * use a lookup table: ADC_TO_CM, and then interpolate between values in the
 * table using a simple ratio applied to the mapped distance:
 * voltage delta from table value[i] or [i-1] / adjacent table value delta.
 *
 * A 5th degree polynomial interpolates the following points from the
 * GP2Y0A02YK0F datasheet plot (Output voltage [V] vs Distance to reflective object L [cm]:
 * x: [2.75, 2.52, 1.249, 1.041, 0.5916, 0.5]
 * y: [15.0, 20.0, 50.0,  60.0,  110.0,  130.0]
 * Which works well to model the behavior of the IR rangefinder. But it is
 * computationally inefficient without a hardware floating point unit.
 *****************************************************************************/
class InfraredRangefinder : public Rangefinder {

public:

    /*
     * Constructor
     */
    InfraredRangefinder(unsigned char signalPin) {

#if USE_RANGEFINDER_AVERAGING
        _average = new ExponentialMovingAverage();
#endif

        _signalPin = signalPin;
    }

    void readDistance(void) {
        // Read the analog input, and calculate the distance.
        // TODO: determine whether we need to take multiple samples and average.
        _rawValue = analogRead(_signalPin);
        /*
        // This is the ideal representation, but it is slow.
        _distance = (15.0+(-21.7391304348*(x-2.75)+(1.24205956448*(x-2.75)
                *(x-2.52)+(-8.95567203181*(x-2.75)*(x-2.52)*(x-1.249)
                +(14.9659864345*(x-2.75)*(x-2.52)*(x-1.249)*(x-1.041)
                +(-14.1814298562*(x-2.75)*(x-2.52)*(x-1.249)*(x-1.041)*(x-0.5916)))))));
        */
        unsigned char idx = 0;
        short deltaInput = 32000; // ((2 ^ 16) -1 / 2) ish.
        for(int i = 0; i < TABLE_ELEMENT_COUNT; i++) {
            if(abs(ADC_TO_CM[i].volts - _rawValue) < deltaInput) {
                deltaInput = abs(ADC_TO_CM[i].volts - _rawValue);
                idx = i;
                _distance = ADC_TO_CM[i].cm;
            }
        }

        unsigned short deltaAdj = 0;
        if(_rawValue < ADC_TO_CM[idx].volts && idx < TABLE_ELEMENT_COUNT - 1) {
            deltaAdj = ADC_TO_CM[idx].volts - ADC_TO_CM[idx + 1].volts;
            deltaInput = ADC_TO_CM[idx].volts - _rawValue;
            float ratio =  (float)deltaInput / (float)deltaAdj;
            deltaAdj = ADC_TO_CM[idx + 1].cm - ADC_TO_CM[idx].cm;
            _distance = ADC_TO_CM[idx].cm + deltaAdj * ratio;
        } else if(_rawValue > ADC_TO_CM[idx].volts && idx > 0) {
            deltaAdj = ADC_TO_CM[idx - 1].volts - ADC_TO_CM[idx].volts;
            deltaInput = ADC_TO_CM[idx - 1].volts - _rawValue;
            float ratio =  (float)deltaInput / (float)deltaAdj;
            deltaAdj = ADC_TO_CM[idx].cm - ADC_TO_CM[idx - 1].cm;
            _distance = ADC_TO_CM[idx - 1].cm + deltaAdj * ratio;
        }

#if USE_RANGEFINDER_AVERAGING
        _average->addSample(_distance);
#endif

    }

private:

    const unsigned char TABLE_ELEMENT_COUNT = 18;

    struct ADCToCM {
        short volts; // ADC value.
        unsigned char cm;
        ADCToCM(short v, unsigned char c) : volts(v), cm(c) { }
    };

    // Look up table to map analog voltage (in ADC output) to distance (CM).
    // 1023 = 5 volts, 1023.0 / 5 = 204.6 per volt.
    const ADCToCM ADC_TO_CM[18] = {
              ADCToCM(620, 10)  // ?
            , ADCToCM(563, 15)  // 2.75
            , ADCToCM(516, 20)  // 2.52
            , ADCToCM(405, 30)  // 1.98
            , ADCToCM(313, 40)  // 1.53
            , ADCToCM(256, 50)  // 1.249
            , ADCToCM(213, 60)  // 1.041
            , ADCToCM(186, 70)  // 0.91
            , ADCToCM(165, 80)  // 0.805
            , ADCToCM(146, 90)  // 0.716
            , ADCToCM(130, 100) // 0.636
            , ADCToCM(121, 110) // 0.5916
            , ADCToCM(111, 120) // 0.5416
            , ADCToCM(102, 130) // 0.5
            , ADCToCM(94,  140) // 0.4585
            , ADCToCM(90,  150) // 0.4375
            , ADCToCM(88,  160) // ?
            , ADCToCM(86,  180) // ?
    };

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
    SonarRangefinder(unsigned char enablePin, unsigned char signalPin) {
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

#if USE_RANGEFINDER_AVERAGING
        _average = new ExponentialMovingAverage(0.4);
#endif

    }

    void readDistance(void) {
        // Assert the enable pin for 20 microseconds or more, then perform a
        // reading on the signal pin. Return the reading
        digitalWrite(_enablePin, HIGH);
        delayMicroseconds(20);
        int delta = 32000;
        int rawValue = 0;
        _prevRawValue = _rawValue;
        // Very simple mode filter. Pick the measurement closest to the last one.
        for(unsigned char i = 0; i < SAMPLE_COUNT; i++) {
            delay(SAMPLE_DELAY);
            // https://www.arduino.cc/en/Tutorial/AnalogInput
            rawValue = analogRead(_signalPin);
            if(abs(rawValue - _prevRawValue) < delta) {
                delta = abs(rawValue - _prevRawValue);
                _rawValue = rawValue;
            }
        }
        digitalWrite(_enablePin, LOW);
        delay(SAMPLE_DELAY);
        // Convert the raw value into distance (centimeters).
        _distance = (unsigned short)((float)_rawValue / SCALE_FACTOR_CM);

#if USE_RANGEFINDER_AVERAGING
        _average->addSample(_distance);
#endif

    }

private:

    unsigned char _enablePin = 0;

    const unsigned char SAMPLE_COUNT = 1;
    
    // Outputs analog voltage with a scaling factor of (Vcc / 512) per inch.
    // A supply of 5V yields ~9.8mV / inch and 3.3V yields ~6.4mV/inch.
    //const float SCALE_FACTOR_IN = 1023.0 / 512.0;
    // 2.54 centimeters per inch.
    const float SCALE_FACTOR_CM = 1023.0 / 1300.48;
    // Delay enough time for sound to travel up to 645 cm to an object and 
    // be reflected back to the sensor.
    const int SAMPLE_DELAY = 23; // milliseconds.
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
        _speedLeft = SPEED_STOP;
        _speedRight = SPEED_STOP;
    }

    virtual ~CommandInterpreter(void) {};

    /**************************************************************************
     * @brief Read commands from a channel
     *
     * @details The implementation of the reader is interface specific.
     *************************************************************************/
    virtual void readCommands(void) { };

    void printState(void) {
#if DEBUG_PRINT
        String s = "Cmds: " + _name + "\n";
        s = s + "l: " + getSpeedLeft() + "\n";
        s = s + "r: " + getSpeedRight() + "\n";
        s = s + "m: " + getRunMode() + "\n";
        Serial.print(s);
#endif
    }

    /**************************************************************************
     * @brief Get the speed for the left motors.
     *************************************************************************/
    unsigned char getSpeedLeft() {
        return _speedLeft;
    }

    /**************************************************************************
     * @brief Get the speed for the right motors.
     *************************************************************************/
    unsigned char getSpeedRight() {
        return _speedRight;
    }

    /**************************************************************************
     * @brief Get the desired run mode.
     *
     * @returns AUTO_AVOID, EXT_CONTROL, or REMOTE_CONTROL
     *************************************************************************/
    unsigned char getRunMode() {
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

    unsigned char _speedLeft;
    unsigned char _speedRight;
    unsigned char _runMode;

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
        _radio.setCS(SPI_CS);

        if(_radio.initialize(FREQUENCY, MYNODEID, NETWORKID)) {

#if DEBUG_PRINT
            Serial.print("Radio init\n");
#endif

        } else {
            Serial.print("Radio init fail\n");
        }

        _radio.setHighPower(); // Always use this for RFM69HCW
        // By default, pin 2 is used for the interrupt pin.
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
                        _speedLeft = (unsigned char)_radio.DATA[i+1];
                    } else if(command == CMD_SPEED_RIGHT) {
                        _speedRight = (unsigned char)_radio.DATA[i+1];
                    } else if(command == CMD_RUN_MODE) {
                        _runMode = (unsigned char)_radio.DATA[i+1];
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

#if DEBUG_PRINT
                Serial.print("Bad cmd: radio\n");
                Serial.print("Data: ");
                for (byte i = 0; i < _radio.DATALEN; i++) {
                    Serial.print((char)_radio.DATA[i]);
                }
                Serial.print("\nRSSI: ");
                Serial.print(_receiveSignalStrength);
                Serial.print("\n");
#endif

            }
        } else {
            // TODO: if we don't receive packets in a while shut down motion.
            _noDataCounter++;
            if(_noDataCounter > NO_DATA_COUNT_LIMIT) {
                _noDataCounter = NO_DATA_COUNT_LIMIT + 1; // prevent overflow.
                _speedLeft = SPEED_STOP;
                _speedRight = SPEED_STOP;
                _runMode = MODE_MOTION_DISABLED;
#if DEBUG_PRINT
                Serial.print("!Radio\n");
#endif
            }
        }
    }

    bool getRadioEnabled(void) {
        return _noDataCounter < NO_DATA_COUNT_LIMIT;
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
        MODE_MOTION_ENABLED_AUTO = 1,
        MODE_MOTION_ENABLED_REMOTE = 2,
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

    const unsigned char NO_DATA_COUNT_LIMIT = 10;
    RFM69 _radio; // = RFM69(SPI_CS, RFM69_IRQ, true, 0);
    short _receiveSignalStrength = 0;
    unsigned char _noDataCounter = 0;
};

/*****************************************************************************/
/**
 * @brief Serial interface based command interpreter.
 *
 * @details Process received serial data for configuration and control commands.
 *****************************************************************************/
class CommandInterpreterSystem : public CommandInterpreter {

public:
    CommandInterpreterSystem(void) : CommandInterpreter("System") {
        _runMode = MODE_REMOTE_CONTROL;
    }

    void readCommands() {
        bool commandValid = true;
        while(Serial.available() > 0) {
            // Read the incoming bytes:
            unsigned char command = Serial.read();

#if DEBUG_PRINT
            Serial.print("Received command: ");
            Serial.print(command);
            Serial.print(", param: ");
#endif

            int value = -1;
            if(Serial.read() == ':') {
                value = Serial.parseInt();
                if(command == CMD_SPEED_LEFT) {
                    _speedLeft = (unsigned char)value;
                } else if(command == CMD_SPEED_RIGHT) {
                    _speedRight = (unsigned char)value;
                } else if(command == CMD_RUN_MODE) {
                    _runMode = (unsigned char)value;
                } else if(command == CMD_RESET) {
                    Serial.print("\nResetting\n");
                    Serial.flush(); // Make sure this is printed.
                    resetFunc();  //call reset
                } else {
                    commandValid = false;
                }

#if DEBUG_PRINT
                Serial.print(value);
                Serial.print("\n");
#endif

                // Parse user indicator commands and configuration commands.
                if(Serial.read() != ';' && !commandValid) {
                    Serial.print("Bad cmd: sys.\n");
                }
            }
        }
    }

    enum RunModes {
        MODE_REMOTE_CONTROL  = 0,
        MODE_SYS_CONTROL     = 1,
        MODE_AUTO_AVOID      = 2,
        MODE_MANUAL_OVERRIDE = 3,
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
        bool result = true;
        // Set the encoder counts to half of uint32_t.max
        if(_claw->SetEncM1(CTL_ADDRESS, 0x7fffffff) == FAILURE) {
            result = FAILURE;
        }
        if(_claw->SetEncM2(CTL_ADDRESS, 0x7fffffff) == FAILURE) {
            result = FAILURE;
        }
        //Set PID Coefficients, Kp, Ki, Kd,
        if(_claw->SetM1VelocityPID(CTL_ADDRESS, KP, KI, KD, QPPS) == FAILURE) {
            result = FAILURE;
        }
        if(_claw->SetM2VelocityPID(CTL_ADDRESS, KP, KI, KD, QPPS) == FAILURE) {
            result = FAILURE;
        }
        // Set Acceleration: don't really need this now since we set
        // acceleration when we set speed.
        if(_claw->SetM1DefaultAccel(CTL_ADDRESS, ACCELERATION) == FAILURE) {
            result = FAILURE;
        }
        if(_claw->SetM2DefaultAccel(CTL_ADDRESS, ACCELERATION) == FAILURE) {
            result = FAILURE;
        }
        if(result == FAILURE) {

#if DEBUG_PRINT
            Serial.print("Motor fail\n");
#endif

        }
    }

    void readEncoderCounts(void) {
        bool valid = false;
        bool readSuccess = true;
        // TODO: we can probably read the encoders with one call, like this:
        //valid = _claw->ReadEncoders(CTL_ADDRESS, &_encCountsRight, &_encCountsLeft);

        _encCountsLeft = _claw->ReadEncM2(CTL_ADDRESS, &_encStatusM2, &valid);
        if(valid == INVALID) {
            readSuccess = FAILURE;
        }
        _encCountsRight = _claw->ReadEncM1(CTL_ADDRESS, &_encStatusM1, &valid);
        if(valid == INVALID) {
            readSuccess = FAILURE;
        }
        if(readSuccess == FAILURE) {

#if DEBUG_PRINT
             Serial.print("Encoder fail\n");
#endif

        }
    }

    unsigned long getEncoderCountsLeft(void) {
        return _encCountsLeft;
    }

    unsigned long getEncoderCountsRight(void) {
        return _encCountsRight;
    }

    /*
    unsigned char getMotorStatusLeft(void) {
        return _encStatusM2;
    }

    unsigned char getMotorStatusRight(void) {
        return _encStatusM1;
    }
    */

    void setSpeed(unsigned char speedLeft, unsigned char speedRight) {
        // Map the input speed (0 to 255 with 128 being zero) to some range of
        // quadrature pulses per second (QPPS). Any value below 128 will
        // produce a negatively signed speed.
        int qppsSpeedLeft = map(speedLeft, 0, 255, -SPEED_LIMIT, SPEED_LIMIT);
        int qppsSpeedRight = map(speedRight, 0, 255, -SPEED_LIMIT, SPEED_LIMIT);
        // Make sure 128 maps to zero.
        if(speedLeft == SPEED_STOP) {
            qppsSpeedLeft = 0;
        }
        if(speedRight == SPEED_STOP) {
            qppsSpeedRight = 0;
        }
        // The acceleration used here, seems not to be the default one...
        _claw->SpeedAccelM1M2(CTL_ADDRESS, ACCELERATION, qppsSpeedRight, qppsSpeedLeft);
        //_claw->SpeedM2(CTL_ADDRESS, qppsSpeedLeft);
        //_claw->SpeedM1(CTL_ADDRESS, qppsSpeedRight);

        // TODO: see about using: SpeedAccelM1M2()
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
    const int QPPS = 10300;
    const int SPEED_LIMIT = 6000;
    const unsigned int ACCELERATION = 4000;
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

        _commandsSystem = new CommandInterpreterSystem();
        _commandsRadio = new CommandInterpreterRadio();

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
                , MUX_DIGITAL_OUT_S1, MUX_DIGITAL_OUT_S2, _muxAnalogInput->UNUSED);

        _motorController = new MotorController(motorController);
    }

    void pollSensorData(void) {
        // Poll the sonar, IR and other sensors, bumper etc..
        // Interleave sonar IR and bumper to allow sonar audio to dissipate.
        _readSonarLeft();
        _readInfraredLeft();
        _readBumperSwitchStates();
        _readSonarFront();
        _readInfraredRight();
        _readSonarRight();
        // Next get the motor encoder counts.
        _readEncoderCounts();

        // Don't bother getting the back sensors unless we are going back.
        // This shortens the loop timing....
        //if(_speedLeft < SPEED_STOP || _speedRight < SPEED_STOP) {
            _readInfraredBack();
            _readSonarBack();
        //} // This is a bad idea, it creates a dependency problem when reversing.
    }

    ///----------------------------------------------------
    /// @brief Rangefinder accessors. Side effect free.
    ///----------------------------------------------------
    unsigned short getSonarLeft(void) {
        return _sonarLeft->getDistanceAverage();
    }

    unsigned short getSonarFront(void) {
        // Return the front sonar distance in centimeters.
        return _sonarFront->getDistanceAverage();
    }

    unsigned short getSonarRight(void) {
        return _sonarRight->getDistanceAverage();
    }

    unsigned short getSonarBack(void) {
        return _sonarBack->getDistanceAverage();
    }

    ///----------------------------------------------------
    /// Infrared range-finders, distance in centimeters.
    ///----------------------------------------------------
    unsigned short getInfraredLeft(void) {
        return _infraredLeft->getDistanceAverage();
    }

    unsigned short getInfraredRight(void) {
        return _infraredRight->getDistanceAverage();
    }

    unsigned short getInfraredBack(void) {
        return _infraredBack->getDistanceAverage();
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

    unsigned char getSpeedLeft() {
        return _speedLeft;
    }

    unsigned char getSpeedRight() {
        return _speedRight;
    }

    void setTimestamp(unsigned long milliseconds, unsigned short microseconds) {
        _prevMilliseconds = _milliseconds; // Store the previous time-stamp.
        _prevMicroseconds = _microseconds;
        _milliseconds = milliseconds; // Store the current time-stamp.
        _microseconds = microseconds; // Modulo 1000.
        // Now we have delta T in microseconds.
        _deltaTime = ((_milliseconds * 1000) + (unsigned long)_microseconds)
            - ((_prevMilliseconds * 1000) + (unsigned long)_prevMicroseconds);
    }

    unsigned long getMilliseconds(void) {
        return _milliseconds;
    }

    unsigned short getMicroseconds(void) {
        return _microseconds;
    }

    void setSpeed(int speedLeft, int speedRight) {
        _speedLeft = speedLeft;
        _speedRight = speedRight;
        _motorController->setSpeed(speedLeft, speedRight);
    }

    /*
    // If we are not going to use these, we can remove them.
    unsigned char getMotorStatusLeft(void) {
        return _motorController->getMotorStatusLeft();
    }

    unsigned char getMotorStatusRight(void) {
        return _motorController->getMotorStatusRight();
    }
    */

    unsigned char getRunModeRadio(void) {
        return _commandsRadio->getRunMode();
    }

    unsigned char getRadioEnabled(void) {
        return _commandsRadio->getRadioEnabled();
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
                || (_commandsRadio->getRunMode()
                    == _commandsRadio->MODE_MOTION_ENABLED_REMOTE)
                || (_commandsSystem->getRunMode()
                    == _commandsSystem->MODE_MANUAL_OVERRIDE);

        if(!motionEnable) {
            digitalWrite(MOTOR_ENABLE, LOW);
            setSpeed(SPEED_STOP, SPEED_STOP);
        } else {
            // Enable the motor controller motor battery, via relay output.
            digitalWrite(MOTOR_ENABLE, HIGH);

            if(((_commandsSystem->getRunMode()
                    == _commandsSystem->MODE_SYS_CONTROL)
                    && (_commandsRadio->getRunMode()
                        == _commandsRadio->MODE_MOTION_ENABLED_AUTO))
                    || (_commandsSystem->getRunMode()
                        == _commandsSystem->MODE_MANUAL_OVERRIDE)) {

                _speedLeft = _commandsSystem->getSpeedLeft();
                _speedRight = _commandsSystem->getSpeedRight();
                setSpeed(_speedLeft, _speedRight);
            } else if(_commandsRadio->getRunMode()
                    == _commandsRadio->MODE_MOTION_ENABLED_REMOTE) {

                _speedLeft = _commandsRadio->getSpeedLeft();
                _speedRight = _commandsRadio->getSpeedRight();
                setSpeed(_speedLeft, _speedRight);
            }
        }

        return motionEnable && _commandsSystem->getRunMode()
                == _commandsSystem->MODE_AUTO_AVOID;
    }

    void printLoopTiming(void) {

#if DEBUG_PRINT
        Serial.print("Loop: ");
        Serial.print(_deltaTime / 1000);
        Serial.print(" mS\n");
#endif

    }

    unsigned short getDeltaTime(void) {
        return _deltaTime / 1000;
    }

private:

    void _readSonarLeft(void) {
        // Ping the front sonar and get the distance in centimeters.
        _setSonarMux(C0_SONAR_ENABLE_LEFT, C0_SONAR_LEFT);
        _sonarLeft->readDistance();
    }

    void _readSonarFront(void) {
        _setSonarMux(C1_SONAR_ENABLE_FRONT, C1_SONAR_FRONT);
        _sonarFront->readDistance();
    }

    void _readSonarRight(void) {
        _setSonarMux(C2_SONAR_ENABLE_RIGHT, C2_SONAR_RIGHT);
        _sonarRight->readDistance();
    }

    void _readSonarBack(void) {
        _setSonarMux(C3_SONAR_ENABLE_BACK, C3_SONAR_BACK);
        _sonarBack->readDistance();
    }

    ///----------------------------------------------------
    /// Infrared range-finder data acquisition.
    ///----------------------------------------------------

    void _readInfraredLeft(void) {
        _muxAnalogInput->selectChannel(C4_IR_DIST_LEFT);
        _infraredLeft->readDistance();
    }

    void _readInfraredRight(void) {
        _muxAnalogInput->selectChannel(C5_IR_DIST_RIGHT);
        _infraredRight->readDistance();
    }

    void _readInfraredBack(void) {
        _muxAnalogInput->selectChannel(C6_IR_DIST_BACK);
        _infraredBack->readDistance();
    }

    void _readBumperSwitchStates(void) {
        _muxAnalogInput->selectChannel(C7_BUMPER_LEFT);
        _bumperStateLeft = digitalRead(MUX_ANALOG_IN_SIG);
        _muxAnalogInput->selectChannel(C8_BUMPER_RIGHT);
        _bumperStateRight = digitalRead(MUX_ANALOG_IN_SIG);
    }

    void _readEncoderCounts(void) {
        // Read the encoder counts from the motor controller.
        _motorController->readEncoderCounts();
    }

    void _checkCommandsSystem(void) {
        _commandsSystem->readCommands();
    }

    void _checkCommandsRadio(void) {
        _commandsRadio->readCommands();
    }

    void _setSonarMux(unsigned char enablePin, unsigned char signalPin) {
        _muxDigitalOutput->selectChannel(enablePin);
        _muxAnalogInput->selectChannel(signalPin);
    }

    SonarRangefinder* _sonarLeft;
    SonarRangefinder* _sonarFront;
    SonarRangefinder* _sonarRight;
    SonarRangefinder* _sonarBack;

    InfraredRangefinder* _infraredLeft;
    InfraredRangefinder* _infraredRight;
    InfraredRangefinder* _infraredBack;

    // Multiplexer for analog input signals such as distance sensors, etc..
    AnalogMultiplexer* _muxAnalogInput;
    // Multiplexer for analog outputs, but we are only using it for digital.
    AnalogMultiplexer* _muxDigitalOutput;

    MotorController* _motorController; // Wraps the RoboClaw.

    unsigned long _milliseconds;
    unsigned short _microseconds;

    unsigned long _prevMilliseconds;
    unsigned short _prevMicroseconds;

    unsigned long _deltaTime = 0; // Time in microseconds between now an previous.

    CommandInterpreterSystem* _commandsSystem;
    CommandInterpreterRadio* _commandsRadio;

    bool _bumperStateLeft = false;
    bool _bumperStateRight = false;

    unsigned char _speedLeft = SPEED_STOP;
    unsigned char _speedRight = SPEED_STOP;
};

/*
 * Auto avoid algorithm:
 * Idea: veer away from detected objects. When too close to an object to avoid
 * it while moving forward, go in reverse.
 *
 * Proportionally turn away from an object based on the left right distances
 * Modulate the speed based on the nearest detected object .
 */

const unsigned short DIST_THRESHOLD_MAX = 100;
const unsigned short DIST_THRESHOLD_MED = 60;
const unsigned short DIST_THRESHOLD_MIN = 30;

// For IR sensors, nominal range is between medium and maximum.

const unsigned char SPEED_CRAWL  = 135;
const unsigned char SPEED_SLOW   = 145;
const unsigned char SPEED_MEDIUM = 160;
const unsigned char SPEED_FAST   = 180;

static unsigned long moveUntil = 0;
unsigned char autoSpeedLeft = SPEED_STOP;
unsigned char autoSpeedRight = SPEED_STOP;

/*****************************************************************************/
/**
 * @brief Drive around and avoid obstacles using range-finder readings.
 *
 * @details Take commands from the system controller and be sensitive to the
 *     dead-man switch.
 *****************************************************************************/
void processAutoAvoidance(PlatformController* ctrl) {

#if DEBUG_PRINT
    Serial.print("Auto avoiding\n");
#endif

    unsigned short sonarLeftDist = ctrl->getSonarLeft();
    unsigned short sonarFrontDist = ctrl->getSonarFront();
    unsigned short sonarRightDist = ctrl->getSonarRight();
    unsigned short sonarBackDist = ctrl->getSonarBack();
    
    bool sonarMaxGt = sonarFrontDist > DIST_THRESHOLD_MAX
            && sonarLeftDist > DIST_THRESHOLD_MAX
            && sonarRightDist > DIST_THRESHOLD_MAX;

    bool sonarMaxLt = sonarFrontDist <= DIST_THRESHOLD_MAX
            || sonarLeftDist <= DIST_THRESHOLD_MAX
            || sonarRightDist <= DIST_THRESHOLD_MAX;

    bool sonarMedGt = sonarFrontDist > DIST_THRESHOLD_MED
            && sonarLeftDist > DIST_THRESHOLD_MED
            && sonarRightDist > DIST_THRESHOLD_MED;

    bool sonarMedLt = sonarFrontDist <= DIST_THRESHOLD_MED
            || sonarLeftDist <= DIST_THRESHOLD_MED
            || sonarRightDist <= DIST_THRESHOLD_MED;

    bool sonarMinGt = sonarFrontDist > DIST_THRESHOLD_MIN
            && sonarLeftDist > DIST_THRESHOLD_MIN
            && sonarRightDist > DIST_THRESHOLD_MIN;

    bool sonarMinLt = sonarFrontDist <= DIST_THRESHOLD_MIN
            || sonarLeftDist <= DIST_THRESHOLD_MIN
            || sonarRightDist <= DIST_THRESHOLD_MIN;

    // TODO: use IR ranges to make auto-avoidance decisions.
    unsigned short irLeftDist = ctrl->getInfraredLeft();
    unsigned short irRightDist = ctrl->getInfraredRight();
    unsigned short irBackDist = ctrl->getInfraredBack();

    bool leftBlocked = irLeftDist > DIST_THRESHOLD_MED
            || irLeftDist < DIST_THRESHOLD_MIN || ctrl->getBumperSwitchStateLeft();

    bool rightBlocked = irRightDist > DIST_THRESHOLD_MED
            || irRightDist < DIST_THRESHOLD_MIN || ctrl->getBumperSwitchStateRight();

    bool irBackBlocked = irBackDist > DIST_THRESHOLD_MED || irBackDist < DIST_THRESHOLD_MIN;

    // And not within absolute minimum thresholds. 
    if(moveUntil > millis() && ((sonarMinGt && sonarBackDist > DIST_THRESHOLD_MIN)
        || (autoSpeedLeft == SPEED_STOP && autoSpeedRight == SPEED_STOP))) {

        // We are executing some kind of evasive maneuver.
        // Move until the time is up.
        ctrl->setSpeed(autoSpeedLeft, autoSpeedRight);
        return;
    }
    
    if(sonarMaxGt && !leftBlocked && !rightBlocked) {
        // Go fast!
        autoSpeedLeft = SPEED_STOP + sonarRightDist / 5;
        autoSpeedRight = SPEED_STOP + sonarLeftDist / 5;
    } else if(sonarMedGt && !leftBlocked && !rightBlocked) {
        autoSpeedLeft = SPEED_STOP + sonarRightDist / 10;
        autoSpeedRight = SPEED_STOP + sonarLeftDist / 10;
    } else if(sonarMaxLt && sonarMinGt && !leftBlocked && !rightBlocked) {
        moveUntil = millis() + 2000;
        // Use almost zero turning radius, faster forward than in reverse.
        if(sonarLeftDist < sonarRightDist) {
            autoSpeedLeft = SPEED_FAST;
            autoSpeedRight = SPEED_STOP - (SPEED_MEDIUM - SPEED_STOP);
        } else {
            autoSpeedLeft = SPEED_STOP - (SPEED_MEDIUM - SPEED_STOP);
            autoSpeedRight = SPEED_FAST;
        }
    } else if(sonarMedLt && sonarBackDist > DIST_THRESHOLD_MIN
            && !ctrl->getBumperSwitchStateLeft() && !ctrl->getBumperSwitchStateRight()) {

        // Don't zero turning radius if the bumpers are triggered.
        moveUntil = millis() + 2000;
        // Use zero turning radius, equal forward and back. Turn in place.
        if(sonarLeftDist < sonarRightDist) {
            autoSpeedLeft = SPEED_STOP + (SPEED_MEDIUM - SPEED_STOP);
            autoSpeedRight = SPEED_STOP - (SPEED_MEDIUM - SPEED_STOP);
        } else {
            autoSpeedLeft = SPEED_STOP - (SPEED_MEDIUM - SPEED_STOP);
            autoSpeedRight = SPEED_STOP + (SPEED_MEDIUM - SPEED_STOP);
        }
        if(leftBlocked) {
            autoSpeedLeft = SPEED_STOP + (SPEED_MEDIUM - SPEED_STOP);
            autoSpeedRight = SPEED_STOP - (SPEED_MEDIUM - SPEED_STOP);
        } else if(rightBlocked) {
            autoSpeedLeft = SPEED_STOP - (SPEED_MEDIUM - SPEED_STOP);
            autoSpeedRight = SPEED_STOP + (SPEED_MEDIUM - SPEED_STOP);
        }
    } else if(sonarBackDist > DIST_THRESHOLD_MED && !irBackBlocked
            && (sonarMinLt || (leftBlocked || rightBlocked))) {

        moveUntil = millis() + 3000;
        // Go in reverse, turn away from nearest thing.
        if(sonarLeftDist < sonarRightDist) {
            autoSpeedLeft = SPEED_STOP - (SPEED_CRAWL - SPEED_STOP);
            autoSpeedRight = SPEED_STOP - (SPEED_FAST - SPEED_STOP);
        } else {
            autoSpeedLeft = SPEED_STOP - (SPEED_FAST - SPEED_STOP);
            autoSpeedRight = SPEED_STOP - (SPEED_CRAWL - SPEED_STOP);
        }
        if(leftBlocked) {
            autoSpeedLeft = SPEED_STOP - (SPEED_CRAWL - SPEED_STOP);
            autoSpeedRight = SPEED_STOP - (SPEED_FAST - SPEED_STOP);
        } else if(rightBlocked) {
            autoSpeedLeft = SPEED_STOP - (SPEED_FAST - SPEED_STOP);
            autoSpeedRight = SPEED_STOP - (SPEED_CRAWL - SPEED_STOP);
        }
    } else {
        moveUntil = millis() + 250;
        // Stop we can't go anywhere.
        autoSpeedLeft = SPEED_STOP;
        autoSpeedRight = SPEED_STOP;
    }


    ctrl->setSpeed(autoSpeedLeft, autoSpeedRight);
    return;
}

// Format sensor data and other platform data into messages to send to the PC:
void sendSystemMessage(PlatformController* ctrl) {
    String message = String("s:");
    message = message
            + ctrl->getSonarLeft() + ","
            + ctrl->getSonarFront() + ","
            + ctrl->getSonarRight() + ","
            + ctrl->getSonarBack() + ","
            + ctrl->getInfraredLeft() + ","
            + ctrl->getInfraredRight() + ","
            + ctrl->getInfraredBack() + ","
            + ctrl->getBumperSwitchStateLeft() + ","
            + ctrl->getBumperSwitchStateRight() + ","
            + ctrl->getEncoderCounterLeft() + ","
            + ctrl->getEncoderCounterRight() + ","
            + ctrl->getSpeedLeft() + "," // Use these to debug the auto avoid routine.
            + ctrl->getSpeedRight() + ","
            + ctrl->getRunModeRadio() + ","
            + ctrl->getRadioEnabled() + ","
            + ctrl->getMilliseconds() + ","
            + ctrl->getMicroseconds() + "\n";
    Serial.print(message);
}

// Using SoftwareSerial. See limitations of Arduino SoftwareSerial
SoftwareSerial serial(MOTOR_RX, MOTOR_TX);

RoboClaw roboclaw(&serial, 10000);

// Create global reference to the platform controller to use
// in the main loop for reading sensor data and performing motor control.
PlatformController* controller;

// Initialize sensors and platform controller:
void setup() {
    Serial.begin(57600);
    serial.begin(38400); // Roboclaw's serial.
    // Hard Reset the RFM module
    pinMode(RFM69_RST, OUTPUT);
    digitalWrite(RFM69_RST, HIGH);
    delay(100);
    digitalWrite(RFM69_RST, LOW);
    delay(100);
    pinMode(MOTOR_ENABLE, OUTPUT);
    controller = new PlatformController(&roboclaw);

#if SHOW_FREE_MEMORY
    Serial.print("Free RAM: ");
    Serial.print(freeMemory());
    Serial.print("\n");
#endif

}

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
    controller->pollSensorData();
    // Get the time since the program started:

    sendSystemMessage(controller);
    if(controller->processControlInput()) {
        // Auto avoid is selected.
        processAutoAvoidance(controller);
    } else {
        autoSpeedLeft = SPEED_STOP;
        autoSpeedRight = SPEED_STOP;
    }
    controller->setTimestamp(millis(), micros() % 1000);
    controller->printLoopTiming();
    return;
}

