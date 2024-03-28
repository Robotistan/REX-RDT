##############
SumoBot
##############

.. image:: /../_static/sumobot.gif

Thanks to its HC-SR04 distance sensor and line follower sensor, SumoBot detects other objects on the track it is in. Thanks to its line-following sensor, SumoBot detects whether it is inside the track and in this way carries the objects off the track.


SumoBot Arduino C Code
-------------------------------


.. code-block::

    #define SensorLEFT 34   // IR pin
    #define SensorRIGHT 35  // IR pin
    int trigPin = 17;       // Trigger
    int echoPin = 16;       // Echo
    long duration, cm;
    
    #define MotorA1 15
    #define MotorA2 23
    
    #define MotorB1 32
    #define MotorB2 33
    
    #define MotorC1 5
    #define MotorC2 4
    
    #define MotorD1 27
    #define MotorD2 14
    
    
    #define fast 255
    #define mid 150
    #define slow 100
    #define THRESHOLD 3600
    
    void forward() {
      analogWrite(MotorA1, mid);
      analogWrite(MotorA2, 0);
    
      analogWrite(MotorB1, mid);
      analogWrite(MotorB2, 0);
    
      analogWrite(MotorC1, mid);
      analogWrite(MotorC2, 0);
    
      analogWrite(MotorD1, mid);
      analogWrite(MotorD2, 0);
    }
    void slowforward() {
      analogWrite(MotorA1, slow);
      analogWrite(MotorA2, 0);
    
      analogWrite(MotorB1, slow);
      analogWrite(MotorB2, 0);
    
      analogWrite(MotorC1, slow);
      analogWrite(MotorC2, 0);
    
      analogWrite(MotorD1, slow);
      analogWrite(MotorD2, 0);
    }
    void right() {
      analogWrite(MotorA1, mid);
      analogWrite(MotorA2, 0);
    
      analogWrite(MotorB1, mid);
      analogWrite(MotorB2, 0);
    
      analogWrite(MotorC1, 0);
      analogWrite(MotorC2, mid);
    
      analogWrite(MotorD1, 0);
      analogWrite(MotorD2, mid);
    }
    
    void left() {
      analogWrite(MotorA1, 0);
      analogWrite(MotorA2, mid);
    
      analogWrite(MotorB1, 0);
      analogWrite(MotorB2, mid);
    
      analogWrite(MotorC1, mid);
      analogWrite(MotorC2, 0);
    
      analogWrite(MotorD1, mid);
      analogWrite(MotorD2, 0);
    }
    
    void stop() {
      analogWrite(MotorA1, 0);
      analogWrite(MotorA2, 0);
    
      analogWrite(MotorB1, 0);
      analogWrite(MotorB2, 0);
    
      analogWrite(MotorC1, 0);
      analogWrite(MotorC2, 0);
    
      analogWrite(MotorD1, 0);
      analogWrite(MotorD2, 0);
    }
    
    void backward() {
      analogWrite(MotorA1, LOW);
      analogWrite(MotorA2, mid);
    
      analogWrite(MotorB1, LOW);
      analogWrite(MotorB2, mid);
    
      analogWrite(MotorC1, LOW);
      analogWrite(MotorC2, mid);
    
      analogWrite(MotorD1, LOW);
      analogWrite(MotorD2, mid);
    }
    
    void setup() {
      Serial.begin(115200);
    
      pinMode(trigPin, OUTPUT);
      pinMode(echoPin, INPUT);
    
      pinMode(SensorLEFT, INPUT);
      pinMode(SensorRIGHT, INPUT);
    
      pinMode(MotorA1, OUTPUT);
      pinMode(MotorA2, OUTPUT);
    
      pinMode(MotorB1, OUTPUT);
      pinMode(MotorB2, OUTPUT);
    
      pinMode(MotorC1, OUTPUT);
      pinMode(MotorC2, OUTPUT);
    
      pinMode(MotorD1, OUTPUT);
      pinMode(MotorD2, OUTPUT);
    }
    
    void loop() {
      digitalWrite(trigPin, LOW);
      delayMicroseconds(5);
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);
    
      duration = pulseIn(echoPin, HIGH);
      cm = (duration / 2) / 29.1;
      /* Serial.print(cm);
      Serial.print("cm");
      Serial.print("  ");*/
      delay(100);
    
    
      int leftSensor = analogRead(SensorLEFT);
      int rightSensor = analogRead(SensorRIGHT);
    
      /*
      Serial.print("leftSensor: ");
      Serial.print(leftSensor);
      Serial.print("  rightSensor: ");
      Serial.println(rightSensor);
      */
    
      if ((leftSensor < THRESHOLD && rightSensor < THRESHOLD)) {
        stop();
        delay(500);
        backward();
        delay(800);
        left();
        delay(400);
    
      } else if ((cm < 15) && (leftSensor >= THRESHOLD && rightSensor >= THRESHOLD)) {
        forward();
      } else {
        slowforward();
      }
    }


SumoBot MicroPython Code
-------------------------------


.. code-block::

    import machine
    from machine import Pin, ADC, PWM, Timer
    from time import sleep
    import utime
    from rex import HCSR04
    
    #IR pins
    leftSensor = ADC(Pin(34))
    rightSensor = ADC(Pin(35))
    
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
    motor_D1 = PWM(Pin(27))
    motor_D1.duty_u16(0)
    motor_D2 = PWM(Pin(14))
    motor_D2.duty_u16(0)
    
    #speed values
    MotorSpeed = 50000
    
    #HCSR04
    sensor = HCSR04(trigger_pin=17, echo_pin=16, echo_timeout_us=10000)
    
    threshold = 65000
    
    def forward(speed):
       motor_A1.duty_u16(speed)
       motor_A2.duty_u16(0)
    
       motor_B1.duty_u16(speed)
       motor_B2.duty_u16(0)
    
       motor_C1.duty_u16(speed)
       motor_C2.duty_u16(0)
    
       motor_D1.duty_u16(speed)
       motor_D2.duty_u16(0)
    
    def right(speed):
       motor_A1.duty_u16(speed)
       motor_A2.duty_u16(0)
    
       motor_B1.duty_u16(speed)
       motor_B2.duty_u16(0)
    
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
    
    def backward(speed):
       motor_A1.duty_u16(0)
       motor_A2.duty_u16(speed)
    
       motor_B1.duty_u16(0)
       motor_B2.duty_u16(speed)
    
       motor_C1.duty_u16(0)
       motor_C2.duty_u16(speed)
    
       motor_D1.duty_u16(0)
       motor_D2.duty_u16(speed)
    
    def stop():
       motor_A1.duty_u16(0)
       motor_A2.duty_u16(0)
    
       motor_B1.duty_u16(0)
       motor_B2.duty_u16(0)
    
       motor_C1.duty_u16(0)
       motor_C2.duty_u16(0)
    
       motor_D1.duty_u16(0)
       motor_D2.duty_u16(0)
    
    while True:
        distance = sensor.distance_cm()
        #print(distance)
        if distance <= 15:
            leftSensorValue = leftSensor.read_u16()
            rightSensorValue = rightSensor.read_u16()
            #print(leftSensorValue)
            #print(rightSensorValue)
            sleep(0.02)
            if leftSensorValue >= threshold or rightSensorValue >= threshold:
                backward(MotorSpeed)
                sleep(0.5)
            elif leftSensorValue < threshold and rightSensorValue < threshold:
                forward(MotorSpeed)
            else:
                stop()
        else:
            leftSensorValue = leftSensor.read_u16()
            rightSensorValue = rightSensor.read_u16()
            #print(leftSensorValue)
            #print(rightSensorValue)
            sleep(0.02)
            if leftSensorValue >= threshold or rightSensorValue >= threshold:
                backward(MotorSpeed)
                sleep(0.5)
            elif leftSensorValue < threshold and rightSensorValue < threshold:
                left(MotorSpeed)
                sleep(0.1)
                stop()
            else:
                stop()





