#include <Wire.h>   // for I2C 

#define CYP_ADDR 0x08       // For cypress board slave 

#define LS_PIN      2
#define DIR_PIN     3
#define STEP_PIN    4
#define SLEEP_PIN   5
#define M1_PIN      6
#define M0_PIN      7


// Move commands will be followed by parameter in 2 byte write 
// First byte will be command type, second byte will be paramters

// Command Codes from Serial Communicatoin 
#define JOG     0xA0   // Followed by number of steps to jog 
#define HOME    0xA2   // Will be followed by 0x00
#define TEST    0xA4   // Followed by three bytes 1) steps, 2) delay (may need 2 bytes) 
#define ID      0xA6    // Command to ask the arduino to identify itself 



#define END_TEST 0xFFFF // Flag to send over serial marking that test ended 

const uint16_t stepDelay    = 1; 
const uint16_t sample_freq  = 100;   // Hz, for recording data 
const uint8_t  num_cap_read = 1; 

const uint8_t pitch = 2; 
const uint16_t steps_per_rev = 200;   // because 200 steps per rev x 4 microsteps 

uint16_t steps_per_mm;

// Define array for holding capacitance sensor readings 
uint16_t CapReadings[num_cap_read]; 

void setup() 
{
    steps_per_mm = steps_per_rev/pitch;   // = 400
    
	  Serial.begin(115200);

    pinMode(DIR_PIN, OUTPUT);
    pinMode(STEP_PIN, OUTPUT);
    pinMode(SLEEP_PIN, OUTPUT);
    pinMode(M0_PIN, INPUT); // to keep floating 
    
    pinMode(M1_PIN, OUTPUT);
    pinMode(LS_PIN, INPUT_PULLUP);
    //attachInterrupt(digitalPinToInterrupt(LS_PIN), , FALLING);
      
    digitalWrite(SLEEP_PIN, HIGH);
    digitalWrite(DIR_PIN, HIGH);
    digitalWrite(STEP_PIN, LOW);
   
    digitalWrite(M1_PIN, LOW);

	  Wire.begin();  // start I2C -- uses analog pins A4 and A5
	  delay(100);    // chill out for a bit while I2C starts up 
}

void loop() 
{

    if (Serial.available() > 1)   // since 2 byte (or more) writes 
    {
        byte command = Serial.read();
        byte value; 

        //Serial.println(inByte);
        // Could write back some type of check sum -- for now just add one 
        //Serial.write(command + 1);
        //int check = value; 
        //Serial.write(check + 1);

        uint16_t distance_t10;
        uint16_t test_step_delay;
        
        uint16_t high_byte;
        uint16_t low_byte;

        switch (command)
        {
            case JOG:
                value = Serial.read(); 
                jog(value); 
                break;  
            case HOME:
                value = Serial.read(); 
                home();
                break; 
            case TEST:
                // num steps -- two bytes 
                
                
                // wait for 4 bytes to come in 
                while(Serial.available() < 4) {}

              

                low_byte = Serial.read();
                high_byte = Serial.read();         
                distance_t10 = (high_byte << 8) | low_byte;  // This is distance in mm x 10

                
                // test_step_delay -- two bytes 
                low_byte = Serial.read();
                high_byte = Serial.read();       
                test_step_delay = (high_byte << 8) | low_byte; 


                
                
                test(distance_t10, test_step_delay); // Not sure if we want milli or micro for this
                break; 
            case ID:
                value = Serial.read();  // dump the next byte 
                Serial.write(0x55);     // Flag to send back as identifier 
                break;
            default:
                int foo = 1; 
               // maybe write some serial command back here to point out we fucked up 
        }  // end switch 
    } // end iff 
} // end loop 

/***************/
void step(int dir)
{
    if (dir == 1)
    {
      digitalWrite(DIR_PIN, HIGH); 
    } else if (dir == -1)
    {
      digitalWrite(DIR_PIN, LOW); 
    }

    digitalWrite(STEP_PIN, HIGH); 
    digitalWrite(STEP_PIN, LOW); 
    
}


/************/

void test(uint16_t distance_t10, uint16_t test_step_delay)
{
    
    // figure out the frequency we want to sample readings at and what we want to report 

    uint32_t steps_t10 = distance_t10 * steps_per_mm;    
    uint16_t num_steps = steps_t10/10;

   
    uint16_t data;      // for holding data points to be transmitted 

    uint32_t start_time = micros();    // get initial time 
    uint32_t prev_time = start_time; 
    uint32_t cur_time = start_time; 
      
    uint32_t sample_thresh = pow(10,6)/(sample_freq + 0.0); // wrng? 
    uint32_t test_time;
    
    // Going Forward -- 
    for (uint16_t steps = 0; steps < num_steps; steps++)
    {           
        if ( ((cur_time - prev_time)  > sample_thresh) || (steps == 0))  
        {
            prev_time = cur_time; 
            capRead(1);     // TODO: comment back in! update one sensor reading 
            data = CapReadings[0]; 
       
            // fill in all the values and write out over the serial comms -- comm up with some standard for formatting it 

            // Account for the extra delay time associated with doing this to keep the motion smooth 

            test_time = cur_time - start_time;            
            Serial.write(test_time & 0xFF);      // Time since test begain in microseconds -- 4 bytes 
            Serial.write( (test_time >> 8)  & 0xFF );
            Serial.write((test_time >> 16) & 0xFF);
            Serial.write((test_time >> 24));
  
            Serial.write(steps & 0xFF);                      // Steps taken since start -- 2 bytes 
            Serial.write(steps >> 8);  
            
            Serial.write(data & 0xFF);                       // Data -- 2 bytes 
            Serial.write(data >> 8);  

        }
        step(1);
        // wait till step delay time has been reached -- blocking code because I'm lazy and a bad person
        while ( (micros() - cur_time) < test_step_delay) {} // just chill           
        cur_time = micros();    
    }       // End extension 

    delay(500);   

    // Retractions 
    for (uint16_t steps = num_steps; steps > 0; steps--)
    {           
        if ( ((cur_time - prev_time)  > sample_thresh) || (steps == 0))  
        {
            prev_time = cur_time; 
            capRead(1);     // TODO: comment back in! update one sensor reading 
            data = CapReadings[0]; 
       
            // fill in all the values and write out over the serial comms -- comm up with some standard for formatting it 

            // Account for the extra delay time associated with doing this to keep the motion smooth 

            test_time = cur_time - start_time;            
            Serial.write(test_time & 0xFF);      // Time since test begain in microseconds -- 4 bytes 
            Serial.write( (test_time >> 8)  & 0xFF );
            Serial.write((test_time >> 16) & 0xFF);
            Serial.write((test_time >> 24));
  
            Serial.write(steps & 0xFF);                      // Steps taken since start -- 2 bytes 
            Serial.write(steps >> 8);  
            
            Serial.write(data & 0xFF);                       // Data -- 2 bytes 
            Serial.write(data >> 8);  

        }
        step(-1);
        // wait till step delay time has been reached -- blocking code because I'm lazy and a bad person
        while ( (micros() - cur_time) < test_step_delay) {} // just chill           
        cur_time = micros();    
    }       // End Retraction
    
    // Send signal that test is over -- all ones for 8 bytes 
    for (int i = 0; i < 8; i++)
    {
        Serial.write(0xFF);
    }
 


}



/************/

void jog(uint16_t dist)    // will be signed 
{
    int dir;
    
    
    
    if (dist >= 128)
    {
      dir = 1; 
      dist = dist - 128; // % TODO -- add math to fix
    }
    else
    {
      dir = -1; 
      dist = 127 - dist; // % TODO -- add math to fix
    }

    uint32_t num_steps = dist * steps_per_mm; 
    
    
    for (int steps = 0; steps < num_steps; steps++)
    {
      step(dir);
      delay(stepDelay); 
    }

}


void home(void)
{
   while(digitalRead(LS_PIN))
   {
     step(-1);
     delay(stepDelay); 
   }

}



void capRead(uint8_t num_cap_read)
{
    Wire.requestFrom(CYP_ADDR, 2 * num_cap_read);   // 2 bytes per reading  

     //uint8_t buffer_pos;
     for (uint8_t i = 0; i< num_cap_read; i ++)
     {
        //buffer_pos = 2 * i;
        uint16_t reading = (Wire.read() << 8);   // high byte
        reading |= Wire.read();                  // low byte 
        CapReadings[i] = reading;
        //Serial.print("Reading "); Serial.print(i); Serial.print(": ");
        //Serial.println(reading, DEC); 
     }

}


