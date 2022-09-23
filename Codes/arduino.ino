#define echoPin1 3 // attach pin D2 Arduino to pin Echo of HC-SR04
#define trigPin 2 //attach pin D3 Arduino to pin Trig of HC-SR04

#define echoPin2 4

// defines variables
long duration; // variable for the duration of sound wave travel
int distance, distance1; // variable for the distance measurement
int v[20];

void sort(int st, int dr) {
  int mid, i, j, aux;
  i = st; j = dr;
  mid = v[(i + j) / 2];
  while(i <= j) {
    while(v[i] < mid) i++;
    while(v[j] > mid) j--;
    if(i <= j) {
      aux = v[i];
      v[i] = v[j];
      v[j] = aux;
      i++; j--;
    }
  }
  if(st < j) sort(st, j);
  if(i < dr) sort(i, dr);
}

int measureDist(int trig, int echo, int sample_size=5, int sample_wait=25, int max_wait=24000) {
  for( int i = 0; i < sample_size; i++ ) {
    digitalWrite(trig, LOW);
    delayMicroseconds(sample_wait);
    // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);
    // Reads the echoPin, returns the sound wave travel time in microseconds
    duration = pulseIn(echo, HIGH, max_wait);
    // Calculating the distance
    distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
    v[i] = distance;
  }
  sort(0, sample_size - 1);
  return v[sample_size / 2];
}

void setup() {
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin1, INPUT); // Sets the echoPin as an INPUT
  pinMode(echoPin2, INPUT);
  digitalWrite(trigPin, LOW);
  Serial.begin(9600); // // Serial Communication is starting with 9600 of baudrate speed
  Serial.println("Ultrasonic Sensor HC-SR04 Test"); // print some text in Serial Monitor
  Serial.println("with Arduino UNO R3");
}

void loop() {
  Serial.print(measureDist(trigPin, echoPin1));
  Serial.print(" ");
  Serial.println(measureDist(trigPin, echoPin2));
}
