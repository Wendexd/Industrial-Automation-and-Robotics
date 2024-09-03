/*
* Kalman filter with Ultrasonic sensor demo
* Industrial Automation and Robotics, ICTE5005
* Department of Computing, Curtin University of Technology
*/

int triggerPin = 11;
int echoPin = 12;
long delayUS ;
long distanceCM ;

// Parameter Initialization of Kalman Filter.
// Note, variance = standard deviation squared,
// hence the multiplications here.
// Measurement noise ( variance ) at time k,
// representing a standard deviation of 2 cm.
// Given to the filter.
float R = 2 * 2;
// World random movement (variance) from time k -1 to time k.
// In this case, we ’re assuming a standard deviation of 5 cm.
// Given to the filter.
float Q = 5 * 5;
// Posterior estimate of position, output of the filter.
float xk;

// Posterior estimate of variance of position,
// output of the filter.
float Pk;
// Previous posterior estimate of position, initialised
// with initial estimate. Is this a good initial estimate?
float xkm1 = 0;
// Previous estimate of variance of position, initialised
// with initial estimate. Is this a good initial estimate?
float Pkm1 = 15 * 15;
// Observed position at time k, input to the filter.
float zk;
// State transition matrix ( world model ).
// Simply the scalar 1, assume no motion.
float F = 1;
// Control model. No control, this is zero.
float B = 0;
// Control input at time k. No control, this is ignored.
float uk = 0;
// Kalman gain.
float Kk;
// Prior estimate of standard deviation of the state estimate.
float Pk_prior;
// Prior estimate of the position.
float xk_prior;
void setup()
{
  // Start the serial port so we can communicate using
  // the serial monitor. 9600 refers to the baud rate
  // (the speed of data transmission) , 9600 is pretty
  // common and reliable if you ’re just doing text.
  Serial.begin(9600);
  // The trigger pin is an output to send the pulse.
  pinMode(triggerPin, OUTPUT);
  // The echo pin is an input to receive the pulse .
  pinMode(echoPin, INPUT);
  // It ’s a good idea to initialise outputs to
  // something that we know is sensible .
  digitalWrite(triggerPin, LOW);
}
void loop()
{
  // Write a 10 us high pulse to the trigger pin.
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  // Wait for the echo pin to go high. The pulseIn
  // function waits until the pin goes high and returns
  // the number of microseconds it took to do so.
  // Waits at most 3,000 microseconds before giving up
  // (corresponds to a little over 0.5 m distance).
  delayUS = pulseIn(echoPin , HIGH , 3000);
  // Convert the delay into a distance. Note that we halve
  // the delay to get the one -way distance.
  distanceCM = (delayUS / 2) / 29.1;
  // Update of the Kalman filter. See slide 30.
  // Populate the input to the Kalman filter.
  zk = distanceCM;
  // Prediction step.
  // Package hasn’t moved, B*uk term is zero.
  // Our prior estimate of position is our last estimate.
  xk_prior = F * xkm1 + B * uk;
  // Our standard deviation increases by Q to form
  // our prior estimate of the standard deviation.
  Pk_prior = Pkm1 + Q;
  // Measurement update step. Note H = 1 as our measurement is in the
  // same dimensionality as the filter. Also note that if we don’t
  // get a measurement (zk == 0) we don ’t perform a measurement update.
  if ( zk != 0)
  {
    // Compute the Kalman gain.
    Kk = Pk_prior / ( Pk_prior + R);
    // Compute the innovation based on the observation, multiply by the
    // Kalman gain, shift the prior estimate of position by that amount
    // to get the current estimate .
    xk = xk_prior + Kk * ( zk - xk_prior);
    // Compute the updated standard deviation .
    Pk = (1 - Kk ) * Pk_prior;
  }
  else
  {
    xk = xk_prior;
    Pk = Pk_prior;
  }
  // Update the previous values
  xkm1 = xk;
  Pkm1 = Pk;
  // Draw bargraphs. Each space is 1 cm so make sure your
  // window is wide enough to fit it all in!
  int i;
  // Integer versions
  int zki = zk;
  int xki = xk;
  int SDi = sqrt(Pk);
  Serial.print(" Measurement : -10 cm >");
  for (i = -10; i < zki ; i ++)
  {
    Serial . print (" ");
  }
  if ( zk != 0) Serial . print ("|");
  else Serial.print(" "); // No measurement .
  for ( i = zki + 1; i < 60; i ++)
  {
    Serial . print (" ");
  }
  Serial . print ("< 60 cm (");
  Serial . print ( zk );
  Serial . println (" cm)");
  Serial . print (" Estimate : -10 cm >");
  for ( i = -10; i < xki - SDi ; i ++)
  {
    Serial . print (" ");
  }
  for ( i = max ( -10 , xki - SDi ); i < xki ; i ++)
  {
    Serial . print ("-");
  }
  Serial . print ("#");
  for ( i = xki + 1; i < xki + SDi ; i ++)
  {
    Serial . print ("-");
  }
  for ( i = xki + SDi ; i < 60; i ++)
  {
    Serial . print (" ");
  }
  Serial . print ("< 60 cm | xk = ");
  Serial . print ( xk );
  Serial . print (" cm | SD = ");
  Serial . print ( sqrt ( Pk ));
  Serial . print (" cm | Kk = ");
  Serial . println ( Kk );
  // Wait before running again .
  delay (50);
}