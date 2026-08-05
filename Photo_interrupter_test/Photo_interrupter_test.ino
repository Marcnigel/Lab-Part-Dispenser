const int sensorPin = A3; 
int previousValue = -1;   

void setup() {
  Serial.begin(9600);     
  while (!Serial) {
    ; 
  }
  Serial.println("Reading initialized...");
}

void loop() {
  int currentValue = analogRead(sensorPin);

  // Check if the current value is strictly different from the last one
  //if (currentValue >= 1000) {
    
    Serial.print("DETECTED AT: ");
    Serial.println(currentValue);

    // Update previous value
    //previousValue = currentValue;
  //}

  // Small delay for ADC stability
  delay(100); 
}