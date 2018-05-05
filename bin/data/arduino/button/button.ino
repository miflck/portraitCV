int incomingByte = 0;   // for incoming serial data


// constants won't change. They're used here to set pin numbers:
const int buttonPin = 13;     // the number of the pushbutton pin

// variables will change:
int buttonState = 0;         // current state of the button
int lastButtonState = 0;     // previous state of the button


unsigned long firstTime; // how long since the button was first pressed
long millis_held;    // How long the button was held (milliseconds)





int inval = 0;


void setup() {
  Serial.begin(9600);
  // initialize the LED pin as an output:
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);

  


  Serial.println(F("----------SETUP----------"));


}

void loop() {


  // Listen for Input
 /* while ( (Serial.available() > 0) )
  {
    inval = Serial.parseInt();
    if (Serial.read() == '\n') {
      Serial.println("");
      Serial.print(inval);
      
    }
  }*/


  // read the state of the pushbutton value:
  buttonState = digitalRead(buttonPin);
  //Serial.println(buttonState);
  if (buttonState != lastButtonState) {
    // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
    if (buttonState == HIGH) {
      // turn LED on:
      Serial.println(buttonState);
      firstTime = millis();
    } 
    // Delay a little bit to avoid bouncing
    delay(50);
  }

  if (buttonState == HIGH) {
    millis_held = (millis() - firstTime);
    if (millis_held > 5000) {
      Serial.println("-1");
      firstTime = millis();
    }
  }

  // save the current state as the last state, for next time through the loop
  lastButtonState = buttonState;

}

