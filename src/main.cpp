/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/vs-code-platformio-ide-esp32-esp8266-arduino/
*********/

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <DFRobotDFPlayerMini.h>
#include <FastLed.h>

#define PIN_DFPLAYER_RX 16
#define PIN_DFPLAYER_TX 17
#define PIN_PAUSE 26
#define PIN_PLAY 32
#define PIN_PREVIOUS 33
#define PIN_NEXT 25
#define PIN_ACTIONNEUR1 23
#define RING_ACTIONNEUR1 15
#define PIN_ACTIONNEUR2 22
#define RING_ACTIONNEUR2 2
#define RING_BOX 4
#define NUM_LEDS 12

struct Button {
    const uint8_t PIN;
    bool pressed;
};

Button buttonPause = {PIN_PAUSE, false};
Button play = {PIN_PLAY, false};
Button previous = {PIN_PREVIOUS, false};
Button next = {PIN_NEXT, false};
Button actionneur1 = {PIN_ACTIONNEUR1, false};
Button actionneur2 = {PIN_ACTIONNEUR2, false};

SoftwareSerial mySoftwareSerial(PIN_DFPLAYER_RX, PIN_DFPLAYER_TX);
DFRobotDFPlayerMini myDFPlayer ;

void printDetail(uint8_t type, int value);
static unsigned long timer = millis();

CRGB ring1[NUM_LEDS];
CRGB ring2[NUM_LEDS];
CRGB ringBox[NUM_LEDS];
CRGB colors[3] = {CRGB::Red, CRGB::Blue, CRGB::Green};
int index_color = 0;

// Called last from the variadic template function
void printLine()
{
  Serial.println();
}

template <typename T, typename... Types>
void printLine(T first, Types... other)
{
  Serial.print(first);
  printLine(other...) ;
}

void IRAM_ATTR isr(void* arg) {
    Button* s = static_cast<Button*>(arg);
    s->pressed = true;
}

void setup() {
  
  pinMode(buttonPause.PIN, INPUT_PULLUP);
  attachInterruptArg(buttonPause.PIN, isr, &buttonPause, FALLING);
  pinMode(play.PIN, INPUT_PULLUP);
  attachInterruptArg(play.PIN, isr, &play, FALLING);
  pinMode(previous.PIN, INPUT_PULLUP);
  attachInterruptArg(previous.PIN, isr, &previous, FALLING);
  pinMode(next.PIN, INPUT_PULLUP);
  attachInterruptArg(next.PIN, isr, &next, FALLING);
  pinMode(actionneur1.PIN, INPUT_PULLUP);
  attachInterruptArg(actionneur1.PIN, isr, &actionneur1, FALLING);
  pinMode(actionneur2.PIN, INPUT_PULLUP);
  attachInterruptArg(actionneur2.PIN, isr, &actionneur2, FALLING);

  FastLED.addLeds<NEOPIXEL, RING_ACTIONNEUR1>(ring1, NUM_LEDS);
  FastLED.addLeds<NEOPIXEL, RING_ACTIONNEUR2>(ring2, NUM_LEDS);
  FastLED.addLeds<NEOPIXEL, RING_BOX>(ringBox, NUM_LEDS);
  
  Serial.begin(115200);

  Serial.println();
  Serial.println(F("DFRobot DFPlayer Mini Demo"));
  Serial.println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));

  mySoftwareSerial.begin(9600, SWSERIAL_8N1, PIN_DFPLAYER_RX, PIN_DFPLAYER_TX, false);
  if (!mySoftwareSerial) { // If the object did not initialize, then its configuration is invalid
    Serial.println("Invalid SoftwareSerial pin configuration, check config"); 
    while (1) { // Don't continue with invalid configuration
      delay (1000);
    }
  } 

  if (!myDFPlayer.begin(mySoftwareSerial, true, false)) {
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!")); 
    while (1) { // Don't continue with invalid configuration
      delay (1000);
    }
  }

  Serial.println(F("DFPlayer Mini online."));
  myDFPlayer.setTimeOut(500); //Set serial communictaion time out 500ms

  //----Set volume----
  myDFPlayer.volume(10);  //Set volume value (0~30).
  myDFPlayer.volumeUp(); //Volume Up
  myDFPlayer.volumeDown(); //Volume Down
  
  //----Set different EQ----
  myDFPlayer.EQ(DFPLAYER_EQ_NORMAL);

  //----Set device we use SD as default----
  myDFPlayer.outputDevice(DFPLAYER_DEVICE_SD);

  //myDFPlayer.play(1);  //Play the first mp3
  myDFPlayer.stop();

  printLine("State: ", myDFPlayer.readState()); //read mp3 state
  printLine("Volume:", myDFPlayer.readVolume()); //read current volume
  printLine("EQ:", myDFPlayer.readEQ()); //read EQ setting
  printLine("FileCounts:", myDFPlayer.readFileCounts()); //read all file counts in SD card
  printLine("FileNumber:", myDFPlayer.readCurrentFileNumber()); //read current play file number
  printLine("FileCountsInFolder 3:", myDFPlayer.readFileCountsInFolder(3)); //read file counts in folder SD:/03
}

void loop() {
  
  if(buttonPause.pressed){
    printLine("Button Pause pressed");
    printLine("State: ", myDFPlayer.readState()); //read mp3 state
    if(myDFPlayer.readState() == 1){ //Playing
      myDFPlayer.pause();
    }else{
      myDFPlayer.start();
    }
    buttonPause.pressed = false;
  }
  if(play.pressed){
    printLine("Button Play pressed");
    myDFPlayer.play();
    play.pressed = false;
  }
  if(previous.pressed){
    printLine("Button Previous pressed");
    myDFPlayer.previous();
    previous.pressed = false;
  }
  if(next.pressed){
    printLine("Button Next pressed");
    myDFPlayer.next();
    next.pressed = false;
  }
  if(actionneur1.pressed){
    printLine("Button Actionneur1 pressed");
    index_color = (index_color + 1) % 3; //change colors
    for(int i=0; i<NUM_LEDS; i++) { // For each pixel in strip...
        ring1[i] = colors[index_color];
      } 
    FastLED.show(); 
    actionneur1.pressed = false;
  }
  if(actionneur2.pressed){
    printLine("Button Actionneur2 pressed");
    index_color = (index_color + 1) % 3; //change colors
    for(int i=0; i<NUM_LEDS; i++) { // For each pixel in strip...
        ring2[i] = colors[index_color];
      } 
    FastLED.show(); 
    actionneur2.pressed = false;
  }

 if (millis() - timer > 1000) { 
    //DO SOME PERIODIC TASKS
    fill_rainbow( ringBox, NUM_LEDS, 250, 7);
    FastLED.show();
    timer = millis();
  }
  
  if (myDFPlayer.available()) {
    printDetail(myDFPlayer.readType(), myDFPlayer.read()); //Print the detail message from DFPlayer to handle different errors and states.
  }

}

void printDetail(uint8_t type, int value){
  switch (type) {
    case TimeOut:
      Serial.println(F("Time Out!"));
      break;
    case WrongStack:
      Serial.println(F("Stack Wrong!"));
      break;
    case DFPlayerCardInserted:
      Serial.println(F("Card Inserted!"));
      break;
    case DFPlayerCardRemoved:
      Serial.println(F("Card Removed!"));
      break;
    case DFPlayerCardOnline:
      Serial.println(F("Card Online!"));
      break;
    case DFPlayerUSBInserted:
      Serial.println("USB Inserted!");
      break;
    case DFPlayerUSBRemoved:
      Serial.println("USB Removed!");
      break;
    case DFPlayerPlayFinished:
      Serial.print(F("Number:"));
      Serial.print(value);
      Serial.println(F(" Play Finished!"));
      break;
    case DFPlayerError:
      Serial.print(F("DFPlayerError:"));
      switch (value) {
        case Busy:
          Serial.println(F("Card not found"));
          break;
        case Sleeping:
          Serial.println(F("Sleeping"));
          break;
        case SerialWrongStack:
          Serial.println(F("Get Wrong Stack"));
          break;
        case CheckSumNotMatch:
          Serial.println(F("Check Sum Not Match"));
          break;
        case FileIndexOut:
          Serial.println(F("File Index Out of Bound"));
          break;
        case FileMismatch:
          Serial.println(F("Cannot Find File"));
          break;
        case Advertise:
          Serial.println(F("In Advertise"));
          break;
        default:
          break;
      }
      break;
    default:
      break;
  }
  
}


