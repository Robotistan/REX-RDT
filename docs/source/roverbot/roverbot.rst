##############
SonicBot
##############

.. image:: /../_static/sonicbot.gif

SonicBot detects objects by emitting sound waves around it thanks to the distance sensor on it and can perform different functions by using its mechanical properties according to the values it senses. You can avoid obstacles by using SonicBot. You can use SonicBot for tasks that require you to detect objects in the environment.


SonicBot Arduino C Code
-------------------------------


.. code-block::

    //"""REX 8in1 Sonic Bot"""
    //Check the web site for Robots https://rex-rdt.readthedocs.io/en/latest/
    
    int trigPin = 17;    // Trigger
    int echoPin = 16;    // Echo
    long duration, cm;
    
    //define speed of motors
    #define SLOW 120
    #define MID 140
    #define FAST 110
    
    //define pins of motors
    #define MotorA1 15
    #define MotorA2 23
    
    #define MotorB1 32
    #define MotorB2 33
    
    #define MotorC1 5
    #define MotorC2 4
    
    #define MotorD1 14
    #define MotorD2 27
    
    //define buzzer pin named "horn"
    #define horn 25
    
    //setting PWM properties
    const int freq = 50;
    const int PWMchannel_1 = 4;
    const int PWMchannel_2 = 5;
    const int PWMchannel_3 = 6;
    const int PWMchannel_4 = 7;
    const int PWMchannel_5 = 8;
    const int PWMchannel_6 = 9;
    const int PWMchannel_7 = 10;
    const int PWMchannel_8 = 11;
    
    const int resolution = 8;
    
    void setup() {
      Serial.begin (115200);   //make sure your Serial Monitor is also set at this baud rate.
    
      pinMode(trigPin, OUTPUT);
      pinMode(echoPin, INPUT);
    
      pinMode(horn, OUTPUT);
    
      pinMode(MotorA1, OUTPUT);
      pinMode(MotorA2, OUTPUT);
    
      pinMode(MotorB1, OUTPUT);
      pinMode(MotorB2, OUTPUT);
    
      pinMode(MotorC1, OUTPUT);
      pinMode(MotorC2, OUTPUT);
    
      pinMode(MotorD1, OUTPUT);
      pinMode(MotorD2, OUTPUT);
    
      ledcSetup(PWMchannel_1, freq, resolution);
      ledcAttachPin(MotorA1, PWMchannel_1);
    
      ledcSetup(PWMchannel_2, freq, resolution);
      ledcAttachPin(MotorA2, PWMchannel_2);
    
      ledcSetup(PWMchannel_3, freq, resolution);
      ledcAttachPin(MotorB1, PWMchannel_3);
    
      ledcSetup(PWMchannel_4, freq, resolution);
      ledcAttachPin(MotorB2, PWMchannel_4);
    
      ledcSetup(PWMchannel_5, freq, resolution);
      ledcAttachPin(MotorC1, PWMchannel_5);
    
      ledcSetup(PWMchannel_6, freq, resolution);
      ledcAttachPin(MotorC2, PWMchannel_6);
    
      ledcSetup(PWMchannel_7, freq, resolution);
      ledcAttachPin(MotorD1, PWMchannel_7);
    
      ledcSetup(PWMchannel_8, freq, resolution);
      ledcAttachPin(MotorD2, PWMchannel_8);
      delay(1500);
    }
    
    void loop() {
      distance();
      if (cm < 8) {
        backward();
        digitalWrite(horn, HIGH);
        delay(100);
        digitalWrite(horn, LOW);
        delay(1);
        left();
        stop();
        delay(100);
      }
      else
      {
        forward();
      }
    }
    
    void forward() { 
      ledcWrite(PWMchannel_1, MID); //MotorA1
      ledcWrite(PWMchannel_2, LOW); //MotorA2
    
      ledcWrite(PWMchannel_3, MID); //MotorB1
      ledcWrite(PWMchannel_4, LOW); //MotorB2
    
      ledcWrite(PWMchannel_5, MID); //MotorC1
      ledcWrite(PWMchannel_6, LOW); //MotorC2
    
      ledcWrite(PWMchannel_7, MID); //MotorD1
      ledcWrite(PWMchannel_8, LOW); //MotorD2
    }
    
    void right() { 
      ledcWrite(PWMchannel_1, LOW); //MotorA1
      ledcWrite(PWMchannel_2, FAST); //MotorA2
    
      ledcWrite(PWMchannel_3, FAST); //MotorB1
      ledcWrite(PWMchannel_4, LOW); //MotorB2
    
      ledcWrite(PWMchannel_5, LOW); //MotorC1
      ledcWrite(PWMchannel_6, FAST); //MotorC2
    
      ledcWrite(PWMchannel_7, LOW); //MotorD1
      ledcWrite(PWMchannel_8, FAST); //MotorD2
      delay(1000);
    }
    
    void left() { 
      ledcWrite(PWMchannel_1, LOW); //MotorA1
      ledcWrite(PWMchannel_2, MID); //MotorA2
    
      ledcWrite(PWMchannel_3, LOW); //MotorB1
      ledcWrite(PWMchannel_4, MID); //MotorB2
    
      ledcWrite(PWMchannel_5, MID); //MotorC1
      ledcWrite(PWMchannel_6, LOW); //MotorC2
    
      ledcWrite(PWMchannel_7, MID); //MotorD1
      ledcWrite(PWMchannel_8, LOW); //MotorD2
      delay(850);
    }
    
    void stop() { 
      ledcWrite(PWMchannel_1, LOW); //MotorA1
      ledcWrite(PWMchannel_2, LOW); //MotorA2
    
      ledcWrite(PWMchannel_3, LOW); //MotorB1
      ledcWrite(PWMchannel_4, LOW); //MotorB2
    
      ledcWrite(PWMchannel_5, LOW); //MotorC1
      ledcWrite(PWMchannel_6, LOW); //MotorC2
    
      ledcWrite(PWMchannel_7, LOW); //MotorD1
      ledcWrite(PWMchannel_8, LOW); //MotorD2
    }
    
    void backward() { 
      ledcWrite(PWMchannel_1, LOW); //MotorA1
      ledcWrite(PWMchannel_2, SLOW); //MotorA2
    
      ledcWrite(PWMchannel_3, LOW); //MotorB1
      ledcWrite(PWMchannel_4, SLOW); //MotorB2
    
      ledcWrite(PWMchannel_5, LOW); //MotorC1
      ledcWrite(PWMchannel_6, SLOW); //MotorC2
    
      ledcWrite(PWMchannel_7, LOW); //MotorD1
      ledcWrite(PWMchannel_8, SLOW); //MotorD2
      delay(200);
    }
    
    void distance() {
      delay(40);
      digitalWrite(trigPin, LOW);
      delayMicroseconds(5);
    
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
    
      digitalWrite(trigPin, LOW);
    
      duration = pulseIn(echoPin, HIGH);
      cm = (duration / 2) / 29.1;
      /* 
      Serial.print(cm);
      Serial.print("cm");
      Serial.println();
      */
    }


SonicBot MicroPython Code
-------------------------------


.. code-block::

    from machine import Pin, ADC, PWM
    import time
    from rex import HCSR04
    
    #motorA
    motor_A1 = PWM(Pin(15))
    motor_A1.duty_u16(0)
    motor_A2 = PWM(Pin(23))
    motor_A2.duty_u16(0)
    
    #motorB
    motor_B1 = PWM(Pin(32))
    motor_B1.duty_u16(0)
    motor_B2 = PWM(Pin(33))
    motor_B2.duty_u16(0)
    
    #motorC
    motor_C1 = PWM(Pin(5))
    motor_C1.duty_u16(0)
    motor_C2 = PWM(Pin(4))
    motor_C2.duty_u16(0)
    
    #motorD
    motor_D1 = PWM(Pin(14))
    motor_D1.duty_u16(0)
    motor_D2 = PWM(Pin(27))
    motor_D2.duty_u16(0)
    
    buzzer = Pin(25, Pin.OUT)
    sensor = HCSR04(trigger_pin=17, echo_pin=16, echo_timeout_us=10000)
    
    #default motor speed
    MotorSpeed = 50000
    
    def forward(speed):
       motor_A1.duty_u16(speed)
       motor_A2.duty_u16(0)
    
       motor_B1.duty_u16(speed)
       motor_B2.duty_u16(0)
    
       motor_C1.duty_u16(speed)
       motor_C2.duty_u16(0)
    
       motor_D1.duty_u16(speed)
       motor_D2.duty_u16(0)
    
    def stop():
       motor_A1.duty_u16(0)
       motor_A2.duty_u16(0)
    
       motor_B1.duty_u16(0)
       motor_B2.duty_u16(0)
    
       motor_C1.duty_u16(0)
       motor_C2.duty_u16(0)
    
       motor_D1.duty_u16(0)
       motor_D2.duty_u16(0)
    
    def backward(speed):
       motor_A1.duty_u16(0)
       motor_A2.duty_u16(speed)
    
       motor_B1.duty_u16(0)
       motor_B2.duty_u16(speed)
    
       motor_C1.duty_u16(0)
       motor_C2.duty_u16(speed)
    
       motor_D1.duty_u16(0)
       motor_D2.duty_u16(speed)
    
    def left(speed):
       motor_A1.duty_u16(0)
       motor_A2.duty_u16(speed)
    
       motor_B1.duty_u16(0)
       motor_B2.duty_u16(speed)
    
       motor_C1.duty_u16(speed)
       motor_C2.duty_u16(0)
    
       motor_D1.duty_u16(speed)
       motor_D2.duty_u16(0)
    
    while True:
        distance = sensor.distance_cm()
        print(distance)
        if distance > 12:
            forward(MotorSpeed)
        else:
            backward(MotorSpeed)
            buzzer.value(1)
            time.sleep(0.3)
            buzzer.value(0)
            left(MotorSpeed)
            time.sleep(0.2)
            stop()
        
