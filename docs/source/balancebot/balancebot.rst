###############
BalanceBot
###############

BalanceBot is a special robot that can stay in balance despite physical interventions thanks to the acceleration sensor on the REX board. You can use BalanceBot in situations where you need to carry objects in balance.

.. image:: /../_static/balancebot-1.png


BalanceBot Arduino C Code
-------------------------------


.. code-block::

    //"""REX 8in1 Balance Bot"""
    //Check the web site for Robots https://rex-rdt.readthedocs.io/en/latest/
    
    #include "I2Cdev.h"
    #include "PID_v1.h" 
    #include "MPU6050_6Axis_MotionApps20.h"
    #include "Wire.h"
    
    #define INTERRUPT_PIN 13
    
    #define Motor_A1 15
    #define Motor_A2 23
    #define Motor_C1 5
    #define Motor_C2 4
    
    MPU6050 mpu;
    
    bool dmpReady = false;  // set true if DMP init was successful
    
    uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
    uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
    uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
    uint16_t fifoCount;     // count of all bytes currently in FIFO
    uint8_t fifoBuffer[64]; // FIFO storage buffer
    
    
    // orientation/motion vars
    Quaternion q;           // [w, x, y, z]         quaternion container
    VectorFloat gravity;    // [x, y, z]            gravity vector
    float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
    
    //............set following 4 values for your robot....
    double setpoint = 187.1; //set the value when the bot is perpendicular to ground using serial monitor.(input value)
    double Kp = 8; //Set this value first
    double Kd = 0.23; //Set this value secound
    double Ki = 100; //Finally set this value
    
    
    double input, output;
    PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);
    
    
    volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
    void dmpDataReady()
    {
      mpuInterrupt = true;
    }
    
    void setup() {
      Serial.begin(115200);
      Wire.begin();
    
      Serial.println(F("Initializing I2C devices..."));
      mpu.initialize();
    
      // verify connection
      Serial.println(F("Testing device connections..."));
      Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    
      // load and configure the DMP
      devStatus = mpu.dmpInitialize();
    
      // supply your own gyro offsets here, scaled for min sensitivity (calibration)
      mpu.setXGyroOffset(45);
      mpu.setYGyroOffset(-38);
      mpu.setZGyroOffset(23);
      mpu.setZAccelOffset(1636);
    
      // make sure it worked (returns 0 if so)
      if (devStatus == 0)
      {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
    
        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(INTERRUPT_PIN, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
    
        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;
    
        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
        
        //setup PID
        pid.SetMode(AUTOMATIC);
        pid.SetSampleTime(10);
        pid.SetOutputLimits(-255, 255);
      }
      else
      {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
      }
    
      //Initialise the Motor outpu pins
      pinMode (Motor_A1, OUTPUT);
      pinMode (Motor_A2, OUTPUT);
      pinMode (Motor_C1, OUTPUT);
      pinMode (Motor_C2, OUTPUT);
    
      pinMode(INTERRUPT_PIN, INPUT_PULLUP);
    
      //By default turn off both the motors
      analogWrite(Motor_A1, LOW);
      analogWrite(Motor_A2, LOW);
      analogWrite(Motor_C1, LOW);
      analogWrite(Motor_C2, LOW);
    }
    
    void loop() {
      // if programming failed, don't try to do anything
      if (!dmpReady) return;
    
      // wait for MPU interrupt or extra packet(s) available
      while (!mpuInterrupt && fifoCount < packetSize)
      {
        //no mpu data - performing PID calculations and output to motors
        pid.Compute();
    
        //Print the value of Input and Output on serial monitor to check how it is working.
        Serial.print(input); Serial.print(" =>"); Serial.println(output);
    
        if (input > 120 && input < 230) { //If the Bot is falling
          if (output > 0) //Falling towards front
            Forward(); //Rotate the wheels forward
          else if (output < 0) //Falling towards back
            Reverse(); //Rotate the wheels backward
        }
        else
          Stop(); //Hold the wheels still
      }
    
      // reset interrupt flag and get INT_STATUS byte
      mpuInterrupt = false;
      mpuIntStatus = mpu.getIntStatus();
      // get current FIFO count
      fifoCount = mpu.getFIFOCount();
    
      // check for overflow (this should never happen unless our code is too inefficient)
      if ((mpuIntStatus & 0x10) || fifoCount == 1024)
      {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
        // otherwise, check for DMP data ready interrupt (this should happen frequently)
      }
      else if (mpuIntStatus & 0x02)
      {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    
        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
    
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
    
        mpu.dmpGetQuaternion(&q, fifoBuffer); //get value for q
        mpu.dmpGetGravity(&gravity, &q); //get value for gravity
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); //get value for ypr
        input = ypr[1] * 180 / M_PI + 180;    
        //Serial.println("ypr[1]=");
        //Serial.println(ypr[1]);
      }
    }
    
    void Forward() //Rotate the wheel forward
    {
      analogWrite(Motor_A1, output);
      analogWrite(Motor_A2, 0);
      analogWrite(Motor_C1, output);
      analogWrite(Motor_C2, 0);
      //Serial.print("F"); //Debugging information
    }
    
    
    void Reverse() //Rotate the wheel reverse
    {
      analogWrite(Motor_A1, 0);
      analogWrite(Motor_A2, output * -1);
      analogWrite(Motor_C1, 0);
      analogWrite(Motor_C2, output * -1);
      //Serial.print("R"); //Debugging information
    }
    
    
    void Stop() //Stop both the wheels
    {
      analogWrite(Motor_A1, 0);
      analogWrite(Motor_A2, 0);
      analogWrite(Motor_C1, 0);
      analogWrite(Motor_C2, 0);
      //Serial.print("S"); //Debugging information
    }



BalanceBot Calibration Code
-------------------------------


.. code-block::


    // MPU6050 offset-finder, based on Jeff Rowberg's MPU6050_RAW
    // 2016-10-19 by Robert R. Fenichel (bob@fenichel.net)
    
    // I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class
    // 10/7/2011 by Jeff Rowberg <jeff@rowberg.net>
    // Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
    //
    // Changelog:
    //      2019-07-11 - added PID offset generation at begninning Generates first offsets 
    //                 - in @ 6 seconds and completes with 4 more sets @ 10 seconds
    //                 - then continues with origional 2016 calibration code.
    //      2016-11-25 - added delays to reduce sampling rate to ~200 Hz
    //                   added temporizing printing during long computations
    //      2016-10-25 - requires inequality (Low < Target, High > Target) during expansion
    //                   dynamic speed change when closing in
    //      2016-10-22 - cosmetic changes
    //      2016-10-19 - initial release of IMU_Zero
    //      2013-05-08 - added multiple output formats
    //                 - added seamless Fastwire support
    //      2011-10-07 - initial release of MPU6050_RAW
    
    /* ============================================
    I2Cdev device library code is placed under the MIT license
    Copyright (c) 2011 Jeff Rowberg
    
    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:
    
    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.
    
    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.
    
      If an MPU6050 
          * is an ideal member of its tribe, 
          * is properly warmed up, 
          * is at rest in a neutral position, 
          * is in a location where the pull of gravity is exactly 1g, and 
          * has been loaded with the best possible offsets, 
    then it will report 0 for all accelerations and displacements, except for 
    Z acceleration, for which it will report 16384 (that is, 2^14).  Your device 
    probably won't do quite this well, but good offsets will all get the baseline 
    outputs close to these target values.
    
      Put the MPU6050 on a flat and horizontal surface, and leave it operating for 
    5-10 minutes so its temperature gets stabilized.
    
      Run this program.  A "----- done -----" line will indicate that it has done its best.
    With the current accuracy-related constants (NFast = 1000, NSlow = 10000), it will take 
    a few minutes to get there.
    
      Along the way, it will generate a dozen or so lines of output, showing that for each 
    of the 6 desired offsets, it is 
          * first, trying to find two estimates, one too low and one too high, and
          * then, closing in until the bracket can't be made smaller.
    
      The line just above the "done" line will look something like
        [567,567] --> [-1,2]  [-2223,-2223] --> [0,1] [1131,1132] --> [16374,16404] [155,156] --> [-1,1]  [-25,-24] --> [0,3] [5,6] --> [0,4]
    As will have been shown in interspersed header lines, the six groups making up this
    line describe the optimum offsets for the X acceleration, Y acceleration, Z acceleration,
    X gyro, Y gyro, and Z gyro, respectively.  In the sample shown just above, the trial showed
    that +567 was the best offset for the X acceleration, -2223 was best for Y acceleration, 
    and so on.
    
      The need for the delay between readings (usDelay) was brought to my attention by Nikolaus Doppelhammer.
    ===============================================
    */
    
    // I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
    // for both classes must be in the include path of your project
    #include "I2Cdev.h"
    #include "MPU6050.h"
    
    // Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
    // is used in I2Cdev.h
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        #include "Wire.h"
    #endif
    
    // class default I2C address is 0x68
    // specific I2C addresses may be passed as a parameter here
    // AD0 low = 0x68 (default for InvenSense evaluation board)
    // AD0 high = 0x69
    MPU6050 accelgyro;
    //MPU6050 accelgyro(0x69); // <-- use for AD0 high
    
    
    const char LBRACKET = '[';
    const char RBRACKET = ']';
    const char COMMA    = ',';
    const char BLANK    = ' ';
    const char PERIOD   = '.';
    
    const int iAx = 0;
    const int iAy = 1;
    const int iAz = 2;
    const int iGx = 3;
    const int iGy = 4;
    const int iGz = 5;
    
    const int usDelay = 3150;   // empirical, to hold sampling to 200 Hz
    const int NFast =  1000;    // the bigger, the better (but slower)
    const int NSlow = 10000;    // ..
    const int LinesBetweenHeaders = 5;
          int LowValue[6];
          int HighValue[6];
          int Smoothed[6];
          int LowOffset[6];
          int HighOffset[6];
          int Target[6];
          int LinesOut;
          int N;
          
    void ForceHeader()
      { LinesOut = 99; }
        
    void GetSmoothed()
      { int16_t RawValue[6];
        int i;
        long Sums[6];
        for (i = iAx; i <= iGz; i++)
          { Sums[i] = 0; }
    //    unsigned long Start = micros();
    
        for (i = 1; i <= N; i++)
          { // get sums
            accelgyro.getMotion6(&RawValue[iAx], &RawValue[iAy], &RawValue[iAz], 
                                 &RawValue[iGx], &RawValue[iGy], &RawValue[iGz]);
            if ((i % 500) == 0)
              Serial.print(PERIOD);
            delayMicroseconds(usDelay);
            for (int j = iAx; j <= iGz; j++)
              Sums[j] = Sums[j] + RawValue[j];
          } // get sums
    //    unsigned long usForN = micros() - Start;
    //    Serial.print(" reading at ");
    //    Serial.print(1000000/((usForN+N/2)/N));
    //    Serial.println(" Hz");
        for (i = iAx; i <= iGz; i++)
          { Smoothed[i] = (Sums[i] + N/2) / N ; }
      } // GetSmoothed
    
    void Initialize()
      {
        // join I2C bus (I2Cdev library doesn't do this automatically)
        #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
            Wire.begin();
        #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
            Fastwire::setup(400, true);
        #endif
    
        Serial.begin(9600);
    
        // initialize device
        Serial.println("Initializing I2C devices...");
        accelgyro.initialize();
    
        // verify connection
        Serial.println("Testing device connections...");
        Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
        Serial.println("PID tuning Each Dot = 100 readings");
      /*A tidbit on how PID (PI actually) tuning works. 
        When we change the offset in the MPU6050 we can get instant results. This allows us to use Proportional and 
        integral of the PID to discover the ideal offsets. Integral is the key to discovering these offsets, Integral 
        uses the error from set-point (set-point is zero), it takes a fraction of this error (error * ki) and adds it 
        to the integral value. Each reading narrows the error down to the desired offset. The greater the error from 
        set-point, the more we adjust the integral value. The proportional does its part by hiding the noise from the 
        integral math. The Derivative is not used because of the noise and because the sensor is stationary. With the 
        noise removed the integral value lands on a solid offset after just 600 readings. At the end of each set of 100 
        readings, the integral value is used for the actual offsets and the last proportional reading is ignored due to 
        the fact it reacts to any noise.
      */
            accelgyro.CalibrateAccel(6);
            accelgyro.CalibrateGyro(6);
            Serial.println("\nat 600 Readings");
            accelgyro.PrintActiveOffsets();
            Serial.println();
            accelgyro.CalibrateAccel(1);
            accelgyro.CalibrateGyro(1);
            Serial.println("700 Total Readings");
            accelgyro.PrintActiveOffsets();
            Serial.println();
            accelgyro.CalibrateAccel(1);
            accelgyro.CalibrateGyro(1);
            Serial.println("800 Total Readings");
            accelgyro.PrintActiveOffsets();
            Serial.println();
            accelgyro.CalibrateAccel(1);
            accelgyro.CalibrateGyro(1);
            Serial.println("900 Total Readings");
            accelgyro.PrintActiveOffsets();
            Serial.println();    
            accelgyro.CalibrateAccel(1);
            accelgyro.CalibrateGyro(1);
            Serial.println("1000 Total Readings");
            accelgyro.PrintActiveOffsets();
         Serial.println("\n\n Any of the above offsets will work nice \n\n Lets proof the PID tuning using another method:"); 
      } // Initialize
    
    void SetOffsets(int TheOffsets[6])
      { accelgyro.setXAccelOffset(TheOffsets [iAx]);
        accelgyro.setYAccelOffset(TheOffsets [iAy]);
        accelgyro.setZAccelOffset(TheOffsets [iAz]);
        accelgyro.setXGyroOffset (TheOffsets [iGx]);
        accelgyro.setYGyroOffset (TheOffsets [iGy]);
        accelgyro.setZGyroOffset (TheOffsets [iGz]);
      } // SetOffsets
    
    void ShowProgress()
      { if (LinesOut >= LinesBetweenHeaders)
          { // show header
            Serial.println("\tXAccel\t\t\tYAccel\t\t\t\tZAccel\t\t\tXGyro\t\t\tYGyro\t\t\tZGyro");
            LinesOut = 0;
          } // show header
        Serial.print(BLANK);
        for (int i = iAx; i <= iGz; i++)
          { Serial.print(LBRACKET);
            Serial.print(LowOffset[i]),
            Serial.print(COMMA);
            Serial.print(HighOffset[i]);
            Serial.print("] --> [");
            Serial.print(LowValue[i]);
            Serial.print(COMMA);
            Serial.print(HighValue[i]);
            if (i == iGz)
              { Serial.println(RBRACKET); }
            else
              { Serial.print("]\t"); }
          }
        LinesOut++;
      } // ShowProgress
    
    void PullBracketsIn()
      { boolean AllBracketsNarrow;
        boolean StillWorking;
        int NewOffset[6];
      
        Serial.println("\nclosing in:");
        AllBracketsNarrow = false;
        ForceHeader();
        StillWorking = true;
        while (StillWorking) 
          { StillWorking = false;
            if (AllBracketsNarrow && (N == NFast))
              { SetAveraging(NSlow); }
            else
              { AllBracketsNarrow = true; }// tentative
            for (int i = iAx; i <= iGz; i++)
              { if (HighOffset[i] <= (LowOffset[i]+1))
                  { NewOffset[i] = LowOffset[i]; }
                else
                  { // binary search
                    StillWorking = true;
                    NewOffset[i] = (LowOffset[i] + HighOffset[i]) / 2;
                    if (HighOffset[i] > (LowOffset[i] + 10))
                      { AllBracketsNarrow = false; }
                  } // binary search
              }
            SetOffsets(NewOffset);
            GetSmoothed();
            for (int i = iAx; i <= iGz; i++)
              { // closing in
                if (Smoothed[i] > Target[i])
                  { // use lower half
                    HighOffset[i] = NewOffset[i];
                    HighValue[i] = Smoothed[i];
                  } // use lower half
                else
                  { // use upper half
                    LowOffset[i] = NewOffset[i];
                    LowValue[i] = Smoothed[i];
                  } // use upper half
              } // closing in
            ShowProgress();
          } // still working
       
      } // PullBracketsIn
    
    void PullBracketsOut()
      { boolean Done = false;
        int NextLowOffset[6];
        int NextHighOffset[6];
    
        Serial.println("expanding:");
        ForceHeader();
     
        while (!Done)
          { Done = true;
            SetOffsets(LowOffset);
            GetSmoothed();
            for (int i = iAx; i <= iGz; i++)
              { // got low values
                LowValue[i] = Smoothed[i];
                if (LowValue[i] >= Target[i])
                  { Done = false;
                    NextLowOffset[i] = LowOffset[i] - 1000;
                  }
                else
                  { NextLowOffset[i] = LowOffset[i]; }
              } // got low values
          
            SetOffsets(HighOffset);
            GetSmoothed();
            for (int i = iAx; i <= iGz; i++)
              { // got high values
                HighValue[i] = Smoothed[i];
                if (HighValue[i] <= Target[i])
                  { Done = false;
                    NextHighOffset[i] = HighOffset[i] + 1000;
                  }
                else
                  { NextHighOffset[i] = HighOffset[i]; }
              } // got high values
            ShowProgress();
            for (int i = iAx; i <= iGz; i++)
              { LowOffset[i] = NextLowOffset[i];   // had to wait until ShowProgress done
                HighOffset[i] = NextHighOffset[i]; // ..
              }
         } // keep going
      } // PullBracketsOut
    
    void SetAveraging(int NewN)
      { N = NewN;
        Serial.print("averaging ");
        Serial.print(N);
        Serial.println(" readings each time");
       } // SetAveraging
    
    void setup()
      { Initialize();
        for (int i = iAx; i <= iGz; i++)
          { // set targets and initial guesses
            Target[i] = 0; // must fix for ZAccel 
            HighOffset[i] = 0;
            LowOffset[i] = 0;
          } // set targets and initial guesses
        Target[iAz] = 16384;
        SetAveraging(NFast);
        
        PullBracketsOut();
        PullBracketsIn();
        
        Serial.println("-------------- done --------------");
      } // setup
     
    void loop()
      {
      } // loop
