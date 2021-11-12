#include <Arduino.h>
#include <SoftwareSerial.h>
#include <DFRobotDFPlayerMini.h>
#include <FastLed.h>
#include <neotimer.h>
#include <StateMachine.h>

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
    uint32_t lastDebounceTime;
    bool pressed;
};

void state1();
void state2();
void state3();
void state4();
void state5();
void state6();
bool transitionS1S2();
bool transitionS2S3();
bool transitionS3S4();
bool transitionS4S5();
bool transitionS4S6();
bool transitionS5S4();
bool transitionS5S6();
bool transitionS6S1();

StateMachine machine = StateMachine();
State* S1 = machine.addState(&state1); //WAIT_QUESTION
State* S2 = machine.addState(&state2); //SAY_ANSWER1
State* S3 = machine.addState(&state3); //SAY_ANSWER2
State* S4 = machine.addState(&state4); //REPEAT_ANSWER1
State* S5 = machine.addState(&state5); //REPEAT_ANSWER2
State* S6 = machine.addState(&state6); //ANSWER

Button buttonPause = {PIN_PAUSE, 0, false};
Button play = {PIN_PLAY, 0, false};
Button previous = {PIN_PREVIOUS, 0, false};
Button next = {PIN_NEXT, 0, false};
Button actionneur1 = {PIN_ACTIONNEUR1, 0, false};
Button actionneur2 = {PIN_ACTIONNEUR2, 0, false};

SoftwareSerial mySoftwareSerial(PIN_DFPLAYER_RX, PIN_DFPLAYER_TX);
DFRobotDFPlayerMini myDFPlayer ;

const uint32_t debounceDelay = 500;
Neotimer myTimer2s(2000); //Create a timer at 2s interval
Neotimer myTimer5s(5000); //Create a timer at 5s interval
Neotimer myTimerS2S3(2000); //Create a transition timer of 2s
Neotimer myTimerS3S4(2000);
Neotimer myTimerS4S5(5000);
Neotimer myTimerS5S4(5000);
Neotimer myTimerS6S1(2000);

boolean isDebounce();
void printDetail(uint8_t type, int value);
void clearRing(CRGB* ring);
void clearAllRings();
void setRingColor(CRGB* ring, int num_leds, CRGB color);
void ringColorWaiting(CRGB* ring, int num_leds, CRGB color);
boolean sayYes(CRGB* ring);
boolean sayNo(CRGB* ring);
boolean repeatAnswer(CRGB ring[], int answer, CRGB color);



CRGB ring1[NUM_LEDS];
CRGB ring2[NUM_LEDS];
CRGB ringBox[NUM_LEDS];
CRGB colors[3] = {CRGB::Red, CRGB::Blue, CRGB::Green};
int index_color = 0;

typedef enum {
  OUI_NON,
  ENCORE_FINI,
} Question;

typedef enum {
  NONE,
  OUI,
  NON,
  ENCORE,
  FINI,
} Answer;

Question question = OUI_NON;
Answer  answer = NONE;

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

  delay(2000);
  clearAllRings();
  
  S1->addTransition(&transitionS1S2,S2);
  S2->addTransition(&transitionS2S3,S3);
  S3->addTransition(&transitionS3S4,S4);
  S4->addTransition(&transitionS4S5,S5);
  S4->addTransition(&transitionS4S6,S6);
  S5->addTransition(&transitionS5S6,S6);
    S5->addTransition(&transitionS5S4,S4);
  S6->addTransition(&transitionS6S1,S1);

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
  myDFPlayer.volume(1);  //Set volume value (0~30).
  myDFPlayer.volumeUp(); //Volume Up
  myDFPlayer.volumeDown(); //Volume Down
  //----Set different EQ----
  myDFPlayer.EQ(DFPLAYER_EQ_NORMAL);
  //----Set device we use SD as default----
  myDFPlayer.outputDevice(DFPLAYER_DEVICE_SD);
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
    if(millis() - buttonPause.lastDebounceTime > debounceDelay){
      printLine("Button Pause pressed");
      printLine("State: ", myDFPlayer.readState()); //read mp3 state
      if(myDFPlayer.readState() == 1){ //Playing
        myDFPlayer.pause();
      }else{
        myDFPlayer.start();
      }
      buttonPause.lastDebounceTime = millis();
    }
    buttonPause.pressed = false;
  }
  if(play.pressed){
    if(millis() - play.lastDebounceTime > debounceDelay){
      printLine("Button Play pressed");
      //myDFPlayer.play();
      play.lastDebounceTime = millis();
    }
    play.pressed = false;
  }
  if(previous.pressed){
    if(millis() - previous.lastDebounceTime > debounceDelay){
      printLine("Button Previous pressed");
      myDFPlayer.previous();
      previous.lastDebounceTime = millis();
    }
    previous.pressed = false;
  }
  if(next.pressed){
    if(millis() - next.lastDebounceTime > debounceDelay){
      printLine("Button Next pressed");
      myDFPlayer.next();
      next.lastDebounceTime = millis();
    }
    next.pressed = false;
  }
  if(actionneur1.pressed){

    if(millis() - actionneur1.lastDebounceTime > debounceDelay){

      printLine("Button Actionneur1 pressed");

      if(question == OUI_NON){
        answer = OUI;
      }
          actionneur1.lastDebounceTime = millis(); //// reset the debouncing timer
    }
    actionneur1.pressed = false;
  }
  if(actionneur2.pressed){
    
    if(millis() - actionneur2.lastDebounceTime > debounceDelay){
      printLine("Button Actionneur2 pressed");

      if(question == OUI_NON){
        answer = NON;
      }
      actionneur2.lastDebounceTime = millis(); //// reset the debouncing timer
    }
    actionneur2.pressed = false;
  }
 
  machine.run();
  
  if (myDFPlayer.available()) {
    printDetail(myDFPlayer.readType(), myDFPlayer.read()); //Print the detail message from DFPlayer to handle different errors and states.
  }

}

//=======================================
void state1()
{
  printLine("STATE1: WAIT_QUESTION");
  question = OUI_NON;
}
bool transitionS1S2(){
  return play.pressed;
}
//=======================================
void state2()
{
  printLine("STATE2: SAY_ANSWER 1");
  if(machine.executeOnce)
  {
    clearAllRings();
    sayYes(ring1);
    myTimerS2S3.start();
  }
}
bool transitionS2S3(){
  return myTimerS2S3.done();
}
//=======================================
//=======================================
void state3()
{
  printLine("STATE3: SAY_ANSWER 2");
  if(machine.executeOnce)
  {
    sayNo(ring2);
    myTimerS3S4.start();
  }
}
bool transitionS3S4()
{
  if(myTimerS3S4.done())
  {
    myTimer5s.start();
    return true;
  }else
  {
    return false;
  }
}
//=======================================
void state4()
{
  printLine("STATE4: REPEAT_ANSWER1");
  if(machine.executeOnce)
  {
    repeatAnswer(ring1, OUI, CRGB::Green);
    myTimerS4S5.start();
  }
}
bool transitionS4S6(){
  return (actionneur1.pressed or actionneur2.pressed);
}
bool transitionS4S5(){
  return(myTimerS4S5.done());
}

void state5()
{
  printLine("STATE5: REPEAT_ANSWER2");
  if(machine.executeOnce)
  {
    repeatAnswer(ring2, NON, CRGB::Red);
    myTimerS5S4.start();
  }
}
bool transitionS5S6(){
  return (actionneur1.pressed or actionneur2.pressed);
}
bool transitionS5S4(){
  return(myTimerS5S4.done());
}
//=======================================
void state6()
{
  printLine("STATE5: ANSWER");
  if(machine.executeOnce)
  {
    clearAllRings();
    if(answer == OUI)
    {
      sayYes(ring1);
    }else
    {
      sayNo(ring2);
    }
    myTimerS6S1.start();
  }
  
}
bool transitionS6S1(){
  return myTimerS6S1.done();
}

void setRingColor(CRGB ring[], int num_leds, CRGB color){
  for(int i=0; i<num_leds; i++) { // For each pixel in strip...
    ring[i] = color;
  }
  FastLED.show();
}

void ringColorWaiting(CRGB ring[], int num_leds, CRGB color){
  
  for(int i=0; i<num_leds; i++) { // For each pixel in strip...
    ring[i] = color;
    delay(100);
    FastLED.show();
  }
  FastLED.show();
}

void clearRing(CRGB ring[]){
  
  for(int i=0; i<NUM_LEDS; i++) { // For each pixel in strip...
    ring[i] = 0x00;
  }
  FastLED.show();
}

void clearAllRings()
{
  clearRing(ring1);
  clearRing(ring2);
  clearRing(ringBox);
}

boolean sayYes(CRGB ring[]){
  setRingColor(ring, NUM_LEDS, CRGB::Green);
  myDFPlayer.playFolder(2,OUI); //play oui
  return myDFPlayer.waitAvailable(5000);
}

boolean sayNo(CRGB ring[]){
  setRingColor(ring, NUM_LEDS, CRGB::Red);
  myDFPlayer.playFolder(2,NON); //play oui
  return myDFPlayer.waitAvailable(5000);
}

boolean repeatAnswer(CRGB ring[], int answer, CRGB color)
{
  clearRing(ring);
  ringColorWaiting(ring, NUM_LEDS, color);
  myDFPlayer.playFolder(2,answer); 
  return myDFPlayer.waitAvailable(5000);
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


