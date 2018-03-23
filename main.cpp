#include "mbed.h"
#include "SHA256.h"
#include "rtos.h"
//Photointerrupter input pins
#define I1pin D2
#define I2pin D11
#define I3pin D12

//Incremental encoder input pins
#define CHA   D7
#define CHB   D8

//Motor Drive output pins   //Mask in output byte
#define L1Lpin D4           //0x01
#define L1Hpin D5           //0x02
#define L2Lpin D3           //0x04
#define L2Hpin D6           //0x08
#define L3Lpin D9           //0x10
#define L3Hpin D10          //0x20
RawSerial pc(SERIAL_TX, SERIAL_RX);
//Mapping from sequential drive states to motor phase outputs
/*
State   L1  L2  L3
0       H   -   L
1       -   H   L
2       L   H   -
3       L   -   H
4       -   L   H
5       H   L   -
6       -   -   -
7       -   -   -
*/
//Drive state to output table
const int8_t driveTable[] = {0x12,0x18,0x09,0x21,0x24,0x06,0x00,0x00};

//Mapping from interrupter inputs to sequential rotor states. 0x00 and 0x07 are not valid
const int8_t stateMap[] = {0x07,0x05,0x03,0x04,0x01,0x00,0x02,0x07};
//const int8_t stateMap[] = {0x07,0x01,0x03,0x02,0x05,0x00,0x04,0x07}; //Alternative if phase order of input or drive is reversed

//Phase lead to make motor spin
int8_t lead = 2;                
int8_t intState = 0;
int8_t orState = 0;
uint64_t c;
char newCmd[18];
Timer t;

//Status LED
DigitalOut led1(LED1);

//Photointerrupter inputs
InterruptIn I1(I1pin);
InterruptIn I2(I2pin);
InterruptIn I3(I3pin);
Thread commOutT(osPriorityNormal, 1024);
Thread commInT(osPriorityNormal, 1024);
Thread motorCtrlT (osPriorityNormal, 1024);

volatile uint64_t newKey;
Mutex newKey_mutex;
int32_t MotorTorque;
int32_t motorPosition; 
int32_t velocity; 
float maxspeed;
uint8_t count_iter;
float setposition;
float outposition;
float outspeed;

enum type0 { Nonce, Hashrate, Key, motorpos, motorvelo};

typedef struct {
    type0 mes_type;
    uint32_t data;
} message_t;

Mail<message_t,16> outMessages;
Queue<void, 8> inCharQ;

//Motor Drive outputs
PwmOut L1L(L1Lpin);
DigitalOut L1H(L1Hpin);
PwmOut L2L(L2Lpin);
DigitalOut L2H(L2Hpin);
PwmOut L3L(L3Lpin);
DigitalOut L3H(L3Hpin);

//Set a given drive state
void motorOut(int8_t driveState, uint32_t MotorTorque)
{
    //Lookup the output byte from the drive state.
    int8_t driveOut = driveTable[driveState & 0x07];

    //Turn off first
    if (~driveOut & 0x01) L1L.pulsewidth_us(0);
    if (~driveOut & 0x02) L1H = 1;
    if (~driveOut & 0x04) L2L.pulsewidth_us(0);
    if (~driveOut & 0x08) L2H = 1;
    if (~driveOut & 0x10) L3L.pulsewidth_us(0);
    if (~driveOut & 0x20) L3H = 1;

    //Then turn on
    if (driveOut & 0x01) L1L.pulsewidth_us(MotorTorque);
    if (driveOut & 0x02) L1H = 0;
    if (driveOut & 0x04) L2L.pulsewidth_us(MotorTorque);
    if (driveOut & 0x08) L2H = 0;
    if (driveOut & 0x10) L3L.pulsewidth_us(MotorTorque);
    if (driveOut & 0x20) L3H = 0;
}

//Convert photointerrupter inputs to a rotor state
inline int8_t readRotorState(){
    return stateMap[I1 + 2*I2 + 4*I3];
}

//Basic synchronisation routine
int8_t motorHome(){
    //Put the motor in drive state 0 and wait for it to stabilise
    motorOut(0, 1000);
    wait(2.0);
    //Get the rotor state
    return readRotorState();
}

void ISR_OUT(){
    static int8_t oldRotorState;
    int8_t rotorState = readRotorState();
    //Poll the rotor state and set the motor outputs accordingly to spin the motor 
    motorOut((rotorState-orState+lead+6)%6, MotorTorque); //+6 to make sure the remainder is positive
    if (rotorState - oldRotorState == 5) {motorPosition--;}     
    else if (rotorState - oldRotorState == -5) {motorPosition++;}     
    else motorPosition += (rotorState - oldRotorState); 
    oldRotorState = rotorState;   
}

void commOutFn(){
    while(1) {
        osEvent newEvent = outMessages.get();
        message_t *pMessage = (message_t*)newEvent.value.p;
        pc.printf("Message %d with data 0x%016x\n\r", pMessage->mes_type,pMessage->data);
        outMessages.free(pMessage);
    }
}

void putMessage(type0 t, uint32_t data){
    message_t *pMessage = outMessages.alloc();
    pMessage->mes_type = t;
    pMessage->data = data;
    outMessages.put(pMessage);
}

void serialISR(){
    uint8_t newChar = pc.getc();
    inCharQ.put((void*)newChar);
}


void commInFn(){
    pc.attach(&serialISR);
    int i = 0;
    while(1) {
        osEvent newEvent = inCharQ.get();
        char newChar = (char)newEvent.value.p;
        pc.printf("Newchar: %x\n\r", newChar);
        if( (newChar) == '\r'){
            newCmd[i] = '\0';
            i = 0;
            if (newCmd[0] == 'K'){
                newKey_mutex.lock();
                sscanf(newCmd, "K%x", &newKey); //Decode the command
                }
            else if(newCmd[0] == 'V'){
                sscanf(newCmd, "V%f", &maxspeed);
                if(newCmd[1] == '0'){
                    MotorTorque = 1000;
                }
                }
            else if(newCmd[0] == 'R'){
                sscanf(newCmd, "R%f", &setposition);
                setposition = setposition +motorPosition/5;
                }
            }
        else{
            if(newCmd[0] == 'K' && i == 17){
                pc.printf("Invalid\n\r");
                i = 0;
                }
            else{             
            newCmd[i] = newChar;
            i++;
            }
        }
}
}

void motorCtrlTick(){
    motorCtrlT.signal_set(0x1);
    }

void motorCtrlFn(){     
    Ticker motorCtrlTicker;  
    motorCtrlTicker.attach_us(&motorCtrlTick,50000); 
    count_iter = 0;
    int32_t oldmotorPosition = 0;
    while(1){      
        motorCtrlT.signal_wait(0x1);
        int32_t position = motorPosition; 
        velocity = (position - oldmotorPosition)*20;
        
        if(count_iter % 20 == 0){
         putMessage(motorpos, motorPosition/5);
         putMessage(motorvelo, velocity/5);  
        }
        oldmotorPosition = position;
        count_iter++;

        if(maxspeed != 0){
             
            if((setposition*5-position)>0){
               outspeed = 25*(maxspeed*5 - abs(velocity));
            }
            else if((setposition*5-position) == 0){
               outspeed = 0;
            }
            else{
               outspeed = 25*(abs(velocity) - maxspeed*5);
            }
                     
            outposition = 25*(setposition*5-position) - 7*velocity;
            
            if(velocity < 0){
                if(outspeed >  outposition){
                    MotorTorque = outspeed;
                }
                else{
                    MotorTorque = outposition;
                }
            }
            else{
                if(outspeed > outposition){
                    MotorTorque = outposition;
                }
                else{
                    MotorTorque = outspeed;
                }
            }
            
            if(MotorTorque < 0){
                MotorTorque = -MotorTorque;
                lead = -2;
            }
            else if ( 0 <= MotorTorque <= 1000 ){
                lead = 2;
            }
            
            else{
                MotorTorque = 1000;
            }
        } 
    }
}

//Main
int main()
{
    L1L.period_us(2000);      
    //L1L.write(0.40f); 
    L2L.period_us(2000);      
    //L2L.write(0.40f); 
    L2L.period_us(2000);      
    //L2L.write(0.40f); 
    
    commOutT.start(commOutFn);
    
    //Initialise the serial port
    
    
    commInT.start(commInFn);
    
    
    pc.printf("Hello\n\r");

    //Run the motor synchronisation
    orState = motorHome();

    pc.printf("Rotor origin: %x\n\r",orState);
    //orState is subtracted from future rotor state inputs to align rotor and motor states

    I1.rise(ISR_OUT);
    I2.rise(ISR_OUT);
    I3.rise(ISR_OUT);
    
    I1.fall(ISR_OUT);
    I2.fall(ISR_OUT);
    I3.fall(ISR_OUT);
    SHA256 obj;

    uint8_t sequence[] = {0x45,0x6D,0x62,0x65,0x64,0x64,0x65,0x64, 0x20,0x53,0x79,0x73,0x74,0x65,0x6D,0x73, 0x20,0x61,0x72,0x65,0x20,0x66,0x75,0x6E, 0x20,0x61,0x6E,0x64,0x20,0x64,0x6F,0x20, 0x61,0x77,0x65,0x73,0x6F,0x6D,0x65,0x20, 0x74,0x68,0x69,0x6E,0x67,0x73,0x21,0x20, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
    uint64_t* key = (uint64_t*)((int)sequence + 48);
    uint64_t* nonce = (uint64_t*)((int)sequence + 56);
    uint8_t hash[32];
    

    motorCtrlT.start(motorCtrlFn);
    t.start();
    pc.printf("Start");
    while(1){
        if ( (*key) != newKey){
            (*nonce) = 0;
            (*key) = newKey;
            putMessage(Key, *key);
        }
        obj.computeHash(hash, sequence, 64);
        c++;
        if (!hash[0] && !hash[1]) {
            putMessage(Nonce, *nonce);
            
        }
        (*nonce)++;
        
        
        if( t.read_ms() >= 1000 ) {
            putMessage(Hashrate, c);
            c = 0;
            t.reset();
        }
    }
}

