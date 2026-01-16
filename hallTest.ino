void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);
pinMode(40, INPUT);
pinMode(41, INPUT);
pinMode(42, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
int hallA = digitalRead(42);
int hallB = digitalRead(41);
int hallC = digitalRead(40);

Serial.print("Hall states: ");
Serial.print(hallA);
Serial.print(" ");
Serial.print(hallB);
Serial.print(" ");
Serial.println(hallC);
}

// you need to upload with USB and then to read from serial you connected to the UART port (the left port if the fuse is at the top of the board relative to your FOV)
