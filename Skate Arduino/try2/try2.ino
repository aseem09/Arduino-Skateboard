void setup() {
Serial.begin(9600);
 }

void loop() {
  
      int data= Serial.read(); // reading the data received from the bluetooth module
      
      Serial.println(data);
  
  delay(500);
}
