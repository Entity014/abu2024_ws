#include <Arduino.h>
#define s0 37 // Module pins wiring
#define s1 38
#define s2 39
#define s3 40
#define out 36

int Red = 0, Blue = 0, Green = 0;
int Red2 = 0, Blue2 = 0, Green2 = 0;

void setup()
{
    pinMode(s0, OUTPUT);
    pinMode(s1, OUTPUT);
    pinMode(s2, OUTPUT);
    pinMode(s3, OUTPUT);
    pinMode(out, INPUT);

    Serial.begin(115200); // intialize the serial monitor baud rate

    digitalWrite(s0, HIGH); // Putting S0/S1 on HIGH/HIGH levels means the output frequency scalling is at 100% (recommended)
    digitalWrite(s1, HIGH); // LOW/LOW is off HIGH/LOW is 20% and LOW/HIGH is 2%
}

void GetColors()
{
    digitalWrite(s2, LOW); // S2/S3 levels define which set of photodiodes we are using LOW/LOW is for RED LOW/HIGH is for Blue and HIGH/HIGH is for green
    digitalWrite(s3, LOW);
    Red = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH); // here we wait until "out" go LOW, we start measuring the duration and stops when "out" is HIGH again, if you have trouble with this expression check the bottom of the code
    delay(20);
    digitalWrite(s3, HIGH); // Here we select the other color (set of photodiodes) and measure the other colors value using the same techinque
    Blue = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH);
    delay(20);
    digitalWrite(s2, HIGH);
    Green = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH);
    delay(20);
}

void loop()
{
    GetColors();
    // Red2 = map(Red, 15, 60, 255, 0); // analogWrite generates a PWM signal with 0-255 value (0 is 0V and 255 is 5V), LED_R is the pin where we are generating the signal, 15/60 are the min/max of the "Red" value, try measuring your own ones
    // Blue2 = map(Green, 30, 55, 255, 0);
    // Green2 = map(Blue, 13, 45, 255, 0);

    Serial.print("RGB [");
    Serial.print(Red);
    Serial.print(",");
    Serial.print(Green);
    Serial.print(",");
    Serial.print(Blue);
    Serial.println("]");
}
