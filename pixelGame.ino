#include <Arduino.h>

#include "DFRobot_OLED12864.h" 

class Runnable {
  static Runnable *headRunnable;
  Runnable *nextRunnable;

  public:
    Runnable() {
      nextRunnable = headRunnable;
      headRunnable = this;
    }

    virtual void setup() = 0;
    virtual void loop() = 0;

    static void setupAll() {
      for (Runnable *r = headRunnable; r; r = r->nextRunnable)
        r->setup();
    }

    static void loopAll() {
      for (Runnable *r = headRunnable; r; r = r->nextRunnable)
        r->loop();
    }
};

Runnable *Runnable::headRunnable = NULL;

class LED: public Runnable {
  const byte powerOutPin;

  public:
    boolean isOn;

    LED(byte powerOutAttach) :
      powerOutPin(powerOutAttach) {
      }
    
    void setup() {
      pinMode(powerOutPin, OUTPUT);
      isOn = false;
      digitalWrite(powerOutPin, 0);
    }

    void loop() {
      
    }

    void powerToggle() {
      if (isOn) {
        digitalWrite(powerOutPin, 0);
        isOn = false;
      } else {
        digitalWrite(powerOutPin, 1);
        delay(50);
        isOn = true;
      }
    }


};

class Button: public Runnable {
  const byte pin;
  int state;
  unsigned long buttonDownMs;

  protected:
    virtual void press() = 0;

  public:
    Button(byte attachTo) :
      pin(attachTo)
    {
    }

    void setup() {
      pinMode(pin, INPUT_PULLUP);
      state = LOW;
    }

    void loop() {
      int prevState = state;
      state = digitalRead(pin);
      if (prevState == HIGH && state == LOW) {
        buttonDownMs = millis();
      } else if (prevState == LOW && state == HIGH) {
        if (millis() - buttonDownMs < 50) {

        } else {
          press();
        }
      }
    }

};

class LEDControlButton: public Button {
  LED &light;
  
  public:
    LEDControlButton(byte attachToPin, LED &attachToLED) :
    Button(attachToPin),
    light(attachToLED) {
    }
  protected:
    void press() {
      // button press
      light.powerToggle();
    }
};
  // Initialize the OLED display using Wire library
  // DFRobot_OLED12864  display(0x3c);

class OLED: public DFRobot_OLED12864, public Runnable {

  public:
    OLED() : DFRobot_OLED12864(0x3c) {}

    void setup() {
      // Initialising the UI will init the display too.
      init();
      flipScreenVertically();
    }

    void loop() {
      display();
      clearOLED();
    }

    void clearOLED() {
      clear();
    }

    void drawPixel(int pixelPositionX, int pixelPositionY) {
      drawRect(pixelPositionX, pixelPositionY, 2, 2);
    }

    void drawRoundObject(int positionX, int positionY, int16_t radius) {
      drawCircle(positionX, positionY, radius);
    }

    void drawCollidedPixel(int positionX, int positionY) {
      drawRect(positionX, positionY, 5, 5);
    }

    void drawCollisions(int collisions) {
      // int y = 120 - (20 * (collisions / 10));
      int y = 120 - collisions;
      drawLine(y, 60, 120, 60);
      if (collisions > 120) {
        fillRect(80, 25, 20, 20);
      }
    }

    void drawTime(float time, float maxTime) {
      float yFloat = 120 - (120 * (time / maxTime));
      int y = (int) yFloat;
      drawLine(y, 2, 120, 2);
      if (time < 0) {
        fillCircle(50, 25, 15);
      }
    }

};

class Object: public Runnable {
  int positionX;
  int positionY;
  int16_t radius = 3;
  OLED& oled;

  public:
    Object(OLED &attachToOLED) : oled(attachToOLED) {}

    void setup() {
      positionX = -200 + random(0, 50);
      positionY = random(0, 64);
    }

    void loop() {
      if (positionX < 200) {
        positionX++;
        drawObject();
      } else {
        setup();
      }
    }

    void drawObject() {
      oled.drawRoundObject(positionX, positionY, radius);
    }

    int getPositionX() {
      return positionX;
    }

    int getPositionY() {
      return positionY;
    }

};

class Player: public Runnable {
  int positionX;
  int positionY;
  int collisions = 0;
  float time = 5000;
  float maxTime = 5000;
  OLED& oled;

  protected:

  public:
    Player(int startPositionX, int startPositionY, OLED& attachToOLED) :
      positionX(startPositionX),
      positionY(startPositionY),
      oled(attachToOLED) { 
      }
    
    void setup() {}

    void loop() {
      time--;
      checkIfPostionIsInBound();
      updateOLED();
    }

    void changeXPostion(int newPositionX) {
      positionX = newPositionX;
      checkIfPostionIsInBound();
      updateOLED();
    }

    void changeYPostion(int newPositionY) {
      positionY = newPositionY;
      checkIfPostionIsInBound();
      updateOLED();
    }

    void incrementXPosition() {
      positionX++;
      checkIfPostionIsInBound();
      updateOLED();
    }

    void decrementXPosition() {
      positionX--;
      checkIfPostionIsInBound();
      updateOLED();
    }

    void incrementYPosition() {
      positionY++;
      checkIfPostionIsInBound();
      updateOLED();
    }

    void decrementYPosition() {
      positionY--;
      checkIfPostionIsInBound();
      updateOLED();
    }

    int getPositionX() {
      return positionX;
    }

    int getPositionY() {
      return positionY;
    }

    void checkIfPostionIsInBound() {

      if (positionX > 126) {
        positionX = 0;
      } else if (positionX < 0) {
        positionX = 126;
      }
      if (positionY > 62) {
        positionY = 0;
      } else if (positionY < 0) {
        positionY = 62;
      }
      
    }

    void updateOLED() {
      oled.drawPixel(positionX, positionY);
      oled.drawTime(time, maxTime);
      oled.drawCollisions(collisions);
    }

    void showCollision() {
      oled.drawCollidedPixel(positionX, positionY);
      collisions++;
    }

};

class Accelerometer: public Runnable {
  int8_t a = 0, b = 0, c = 0;

  public:
    void setup() {
      Wire.beginTransmission(0x0B); // address of the accelerometer
      Wire.write(0x20);
      Wire.write(0x05);
    }

    void loop() {
      AccelerometerInit();
    }

    int8_t getA() {
      return a;
    }

    int8_t getB() {
      return b;
    }

    int8_t getC() {
      return c;
    }

  void AccelerometerInit(void) {
    byte Version[3] = {0};
    int8_t x_data = 0, y_data = 0, z_data = 0;
    Wire.beginTransmission(0x0B); // address of the accelerometer 
    // reset the accelerometer 
    Wire.write(0x04); // Y data
    Wire.endTransmission(); 
    Wire.requestFrom(0x0B,1);    // request 6 bytes from slave device #2
    while(Wire.available())    // slave may send less than requested
    { 
      Version[0] = Wire.read(); // receive a byte as characte
    }  
    x_data = (int8_t)Version[0] >> 2;
    
    Wire.beginTransmission(0x0B); // address of the accelerometer 
    // reset the accelerometer 
    Wire.write(0x06); // Y data
    Wire.endTransmission(); 
    Wire.requestFrom(0x0B,1);    // request 6 bytes from slave device #2
    while(Wire.available())    // slave may send less than requested
    { 
      Version[1] = Wire.read(); // receive a byte as characte
    }  
    y_data = (int8_t)Version[1] >> 2;
    
    Wire.beginTransmission(0x0B); // address of the accelerometer 
    // reset the accelerometer 
    Wire.write(0x08); // Y data
    Wire.endTransmission(); 
    Wire.requestFrom(0x0B,1);    // request 6 bytes from slave device #2
    while(Wire.available()) 
    { 
      Version[2] = Wire.read(); // receive a byte as characte
    }
    z_data = (int8_t)Version[2] >> 2; 
    a = x_data;
    b = y_data;
    c = z_data;
    // Serial.print("acc_X = ");
    // Serial.print(a);
    // Serial.print("  ");
    // Serial.print("acc_Y = ");
    // Serial.print(b);
    // Serial.print("  ");
    // Serial.print("acc_Z = ");
    // Serial.println(c);
  }


  
};

class PlayerControlAccelerometer: public Runnable {
  Player &player;
  Accelerometer &accelerometer;

  public:
    PlayerControlAccelerometer(Player &attachToPlayer, Accelerometer &attachToAccelerometer) : 
    player(attachToPlayer),
    accelerometer(attachToAccelerometer) {
    }

    void setup() {}

    void loop() {
      checkXAccerleration();
      checkYAccerleration();
    }

    void checkXAccerleration() {
      int8_t a = accelerometer.getA();
      int pixelPositionX = player.getPositionX();

      if (a > 5) {
        if (a > 8) {
          if (a > 10) {
            if(a > 12) {
              if (a > 15)
              {
                pixelPositionX = pixelPositionX - 3;
              }
              
              pixelPositionX--;
            }
            pixelPositionX--;
          }
          pixelPositionX--;
        }
        pixelPositionX--;
      } else if (a < -5) {
        if (a < -8) {
          if (a < -10) {
            if (a < -12) {
              if (a < -15) {
                pixelPositionX = pixelPositionX + 3;
              }
              pixelPositionX++;
            }
            pixelPositionX++;
          }
          pixelPositionX++;
        }
        pixelPositionX++;
      }

      player.changeXPostion(pixelPositionX);
    }

    void checkYAccerleration() {
      int8_t b = accelerometer.getB();
      int pixelPositionY = player.getPositionY();


      if (b > 5) {
        if (b > 8) {
          if (b > 10) {
            if(b > 12) {
              if (b > 15)
              {
                pixelPositionY = pixelPositionY + 3;
              }
              
              pixelPositionY++;
            }
            pixelPositionY++;
          }
          pixelPositionY++;
        }
        pixelPositionY++;
      } else if (b < -5) {
        if (b < -8) {
          if (b < -10) {
            if (b < -12) {
              if (b < -15) {
                pixelPositionY = pixelPositionY - 3;
              }
              pixelPositionY--;
            }
            pixelPositionY--;
          }
          pixelPositionY--;
        }
        pixelPositionY--;
      }
      
      player.changeYPostion(pixelPositionY);
    }

};

class ControllerStick: public Runnable {
  #define ADC_SECTION 5
  #define ADC_BIT 600


  uint8_t pin_analogKey;
  enum enum_key_analog {
    key_analog_no,
    key_analog_right,
    key_analog_center,
    key_analog_up,
    key_analog_left,
    key_analog_down,
  } key_analog;

  //uint8_t key_analog = key_analog_no;

  protected:
    virtual void no() = 0;
    virtual void up() = 0;
    virtual void down() = 0;
    virtual void right() = 0;
    virtual void left() = 0;
    virtual void center() = 0;

  public:
  ControllerStick(uint8_t attachToAnalogKey) :
    pin_analogKey(attachToAnalogKey) {
    }

  void setup() {
    
  }

  void loop() {
  key_analog = readKeyAnalog();
  switch(key_analog) {
    case key_analog_no:     no();break;
    case key_analog_up:     up();break;
    case key_analog_down:   
    down();
    break;
    case key_analog_right:
    right();
    break;
    case key_analog_left:   
    left();
    break;
    case key_analog_center: center();break;
  }

  }

  enum_key_analog readKeyAnalog() {
    int adValue = analogRead(pin_analogKey);
    if(adValue > ADC_BIT * (ADC_SECTION * 2 - 1) / (ADC_SECTION * 2)) {
      return key_analog_no;
    }
    else if(adValue > ADC_BIT * (ADC_SECTION * 2 - 3) / (ADC_SECTION * 2)) {
      return key_analog_right;
    }
    else if(adValue > ADC_BIT * (ADC_SECTION * 2 - 5) / (ADC_SECTION * 2)) {
      return key_analog_center;
    }
    else if(adValue > ADC_BIT * (ADC_SECTION * 2 - 7) / (ADC_SECTION * 2)) {
      return key_analog_up;
    }
    else if(adValue > ADC_BIT * (ADC_SECTION * 2 - 9) / (ADC_SECTION * 2)) {
      return key_analog_left;
    }
    else {
      return key_analog_down;
    }
  }

};

class PlayerControllerStick: public ControllerStick {
  Player &player;

  public:
    PlayerControllerStick(uint8_t attachToAnalogKey, Player &attachToPlayer) :
      ControllerStick(attachToAnalogKey),
      player(attachToPlayer) {
      }

  protected:
    void no() {}

    void up() {
      player.decrementYPosition();
    }
    
    void down() {
      player.incrementYPosition();
    }

    void right() {
      player.incrementXPosition();
    }

    void left() {
      player.decrementXPosition();
    }

    void center() {}
};

class CollisionDetection: public Runnable {
  Player &player;
  Object &object;

  public:
  CollisionDetection(Player &attachToPlayer, Object &attachToObject) : 
    player(attachToPlayer),
    object(attachToObject) {

  }

  void setup() {}

  void loop() {
    int positionX = player.getPositionX();
    int positionY = player.getPositionY();
    if (object.getPositionX() >= (positionX - 3) && object.getPositionX() < (positionX + 3)) {
      if (object.getPositionY() >= (positionY - 3) && object.getPositionY() < (positionY + 3)) {
        player.showCollision();
      }
    }
  }

};




LED led0(PIN_LED0);
LED led1(PIN_LED1);
LEDControlButton buttonA(8, led0);
LEDControlButton buttonB(9, led1);
OLED mainOled;
Object object0(mainOled);
Object object1(mainOled);
Object object2(mainOled);
Player pixelPlayer(5, 5, mainOled);
PlayerControllerStick mainPlayerControllerStick(A0, pixelPlayer);
Accelerometer accelerometer;
PlayerControlAccelerometer playerControlAccelerometer(pixelPlayer, accelerometer);

CollisionDetection collisionDetection0(pixelPlayer, object0);
CollisionDetection collisionDetection1(pixelPlayer, object1);
CollisionDetection collisionDetection2(pixelPlayer, object2);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Runnable::setupAll();
  Serial.println("setup complete");
}

void loop() {
  // put your main code here, to run repeatedly: 
  Runnable::loopAll();
}
