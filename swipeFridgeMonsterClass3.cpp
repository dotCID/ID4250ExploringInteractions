/*
	Written by Marien ".CID" Wolthuis for ID4250 - started at 15/1/2014
	
	This is a godclass containing both the code for swipe functionality and basic LED lighting.
	Features:
		- Lights LEDs in the correct paired sequence
		- Reads out the sensor nodes (at TimeToNext/25 intervals)
		- Tracks user progress through the sequence
		- Switches LEDs back to an earlier state if they run ahead of the user
		- Blinks LED 8 and 19 when unlocked
	
	For electronics schematic refer to prototypeElectronicsDesign.png
	
	Numbering of sensor nodes: (front view)
		___________________
		|       1   0       |
		|		.	.		|
		|       .   .       |
		|       .   .       | 
		|       3   2       |
		|5=====/-----\=====4|
		|               ../ |
		|           .../    |
		|      ..../        |
		|7====/------------6|
		¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯
	Sensor nodes should be touched in the order
		Step 0:	1&0
		Step 1:	3&2
		Step 2:	5&4
		Step 3:	5&7
		Step 4:	4&6
	to unlock the fridge.
	
	LED lighting pairs:
				|  GP0 |IODIR0|  GP1 |IODIR1|
	7 + 13	=	| 0x20 | 0x40 | 0x01 | 0x0A |
	6 + 12	=	| 0x80 | 0x40 | 0x04 | 0x0A |
	5 + 11	=	| 0x20 | 0x80 | 0x01 | 0x0C |
	2 + 9	=	| 0x40 | 0x80 | 0x02 | 0x0C |
	4 + 10	=	| 0x40 | 0x20 | 0x02 | 0x09 |
	4 + 14	=	| 0x08 | 0x02 | 0x02 | 0x09 |
	4 + 16	=	| 0x04 | 0x02 | 0x02 | 0x09 |
	1 + 15	=	| 0x04 | 0x08 | 0x04 | 0x09 |
	3 + 17	=	| 0x02 | 0x08 | 0x04 | 0x03 |
	0 + 18	=	| 0x08 | 0x04 | 0x08 | 0x03 |
	8 + 19	=	| 0x82 | 0x24 |		 |		|
	
	Numbering is consistent with the wiring scheme
*/
#include <CapacitiveSensor.h>
#define NODE_SAMPLES 15
#define SIG_PIN 2
#define NODE1_PIN 3
#define NODE2_PIN 4
#define NODE3_PIN 5
#define NODE4_PIN 6
#define NODE5_PIN 7
#define NODE6_PIN 8
#define NODE7_PIN 9
#define NODE8_PIN 10

#include <Wire.h>
#define GP0 0x00
#define GP1 0x01
#define IODIR0 0x06
#define IODIR1 0x07

// values begin
int LEDpairs[20][5] = {	// contains the GP/IODIR and timing values, per LED pair: { GP0, IODIR0, GP1, IODIR1, TimeToNext }
	{ 0x20 , 0x40 , 0x01 , 0x0A, 375 },	// step 1->2: 750ms
	{ 0x80 , 0x40 , 0x04 , 0x0A, 375 },
	{ 0x20 , 0x80 , 0x01 , 0x0C, 350 }, // step 2->3: 700ms
	{ 0x40 , 0x80 , 0x02 , 0x0C, 350 },
	{ 0x40 , 0x20 , 0x02 , 0x09, 300 }, // step 3->4: 1200ms
	{ 0x08 , 0x02 , 0x02 , 0x09, 300 },
	{ 0x04 , 0x02 , 0x02 , 0x09, 300 },
	{ 0x04 , 0x08 , 0x04 , 0x09, 300 }, // step 4->5: 1200ms
	{ 0x02 , 0x08 , 0x04 , 0x03, 300 },
	{ 0x08 , 0x04 , 0x08 , 0x03, 300 },
	{ 0x82 , 0x24 ,	0x00 , 0x00, 300 }  // step 5
};

int nodePairs[5][2] = {	// contains the combinations of pads touched
	{ 1, 0 },
	{ 3, 2 },
	{ 5, 4 },
	{ 5, 7 },
	{ 4, 6 }
};

int touchLEDcorrespondence[5] = {	// touch steps, with in the column corresponding LED pairs AT the nodes (these correspond with loop()->i-1)
	0,
	1,
	3,
	6,
	9
};

const bool FDEBUG = false;						// unlocks full debug prints (readouts, precise values, etc)
const bool DEBUG = true;						// unlocks partial debug prints (state completion)
const bool haltUntilFirstTouch = true;			// halts lighting of LED sequence until first pads are touched

bool started = false;
bool fridgeUnlocked = false;
bool touchedNodes[8] = {false}; 				// these are only flipped to true when nodes are touched
bool stepsCompleted[5] = {false};
bool revertTo0 = false;							// needed to set LEd pair loop to 0 correctly
int previousCycle = -1;							// tracks previously touched pair
int cycleCount = 0;								// tracks loop count

int unlockedLed = 13;							// both onboard LED and external

CapacitiveSensor nodes[8] = {
								CapacitiveSensor(SIG_PIN,NODE1_PIN),
								CapacitiveSensor(SIG_PIN,NODE2_PIN),
								CapacitiveSensor(SIG_PIN,NODE3_PIN),
								CapacitiveSensor(SIG_PIN,NODE4_PIN),
								CapacitiveSensor(SIG_PIN,NODE5_PIN),
								CapacitiveSensor(SIG_PIN,NODE6_PIN),
								CapacitiveSensor(SIG_PIN,NODE7_PIN),
								CapacitiveSensor(SIG_PIN,NODE8_PIN)
							};

// Threshold to compare results with
long node_threshold = 170;	// it seems two simultaneously touched nodes may interfere. a lower threshold may help (120-175?)

void setup() {
	Serial.begin(9600);

	pinMode(unlockedLed, OUTPUT);
	
	Wire.begin();
	Wire.beginTransmission(0x20);
	Wire.write(IODIR0);
	Wire.write(0x00);
	Wire.write(0x00);				// set all pins to output mode
	Wire.endTransmission();
	
	Wire.beginTransmission(0x20);
	Wire.write(GP0);
	Wire.write(0x00);				// set all pins to low
	Wire.write(0x00);
	Wire.endTransmission();	
}

void loop(){
	if(!started){ 
		print("Program active\n\n",0); 
		started=true;
	};
	
	for(int i=0;i<11;i++){ // count over LED pairs
		if(i==0) { 
			if(cycleCount < 4){
				cycleCount++;
			} else reInit();	// reinit after 3 incomplete cycles
		};
		if(checkCompletion() == -1 && haltUntilFirstTouch) { i=0;} // stalls the sequence on position 1 until that is touched
		
		print("Pair ",0); print(i,0); print(" is on.\n",0);
		killLEDs();
		lightLEDpair(i);
				  
		for(int j=0;j<25;j++){ // split delays in about 10-20ms to have steady timing			
			if(fridgeUnlocked) unlock();
			
			checkNodes();
			int completedStep = checkSequence();
			
			if((completedStep >= 0) &&
			   (completedStep != previousCycle) &&
			   (touchLEDcorrespondence[completedStep] < i+1)) { // if the LEDs are ahead of the user, restart at the user's current position
			   
			    previousCycle = completedStep;
				i = touchLEDcorrespondence[completedStep];		// it seems if i=0 here the next loop is executed with i=1
				if(i == 0) revertTo0 = true;
				print("Reverted to pair ",0);print(i,0);print("\n",0);
				break;
			}
			previousCycle = completedStep;
			delay((LEDpairs[i][4]/25));
			
		}
		if(revertTo0){
			revertTo0 = false;
			break;	// break main for loop to get i=0
		}
		if(fridgeUnlocked){ 
			print("Successfully unlocked!\n\n",0);
			lightLEDpair(10);
			
			for(int j=0;j<3;j++){
				delay(2500);
				killLEDs();
				delay(1500);
				lightLEDpair(10);
			};
			
			delay(3000); 
			reInit(); 
			break;
		};
	}
}

int checkCompletion(){
	int totalStepsCompleted = 0;
	for(int i=0;i<5;i++){
		if(stepsCompleted[i]) { totalStepsCompleted++;}
		else break;
	}
	if(totalStepsCompleted == 5){ fridgeUnlocked = true;}
	
	return totalStepsCompleted-1;	// -1 otherwise it does not line up with the nodePairs array
}

void unlock(){
	// unlock code goes here: checking is done in loop() to enable unlocking with f.i. an override code
	digitalWrite(unlockedLed, HIGH);
}

void checkNodes(){
	long results[8];
	for(int i=0;i<8;i++){
		results[i] = nodes[i].capacitiveSensor(NODE_SAMPLES);
		if(results[i] > node_threshold){ touchedNodes[i] = true; cycleCount=0;}; //reset cycle counter so user has enough time to figure things out
		print(results[i],1);
		print("\t",1);
	};
	for(int i=0;i<8;i++){
		if(touchedNodes[i]){ print("1",1); } else { print("0",1); };
	}
	print("\n",1);
}

int checkSequence(){
	int stepComplete = -1;
	
	for(int i=0;i<5;i++){
		if((stepsCompleted[i-1] || i==0) &&	// only check if either i==0 or the previous step was completed
		   (touchedNodes[nodePairs[i][0]] && touchedNodes[nodePairs[i][1]])) {
			stepsCompleted[i] = true;
			stepComplete = i;						// assumes only one step can be completed at a time (since multiple would be physically impossible within a clock cycle)
		}
	}
	
	// reset touchedNodes after this checkcycle
	for(int i=0;i<8;i++){
		if(touchedNodes[i]){touchedNodes[i]=false;}; 
    };
	
	return stepComplete;
}

void lightLEDpair(int i){
	sendMessage(GP0, LEDpairs[i][0], LEDpairs[i][2]);
	sendMessage(IODIR0, LEDpairs[i][1], LEDpairs[i][3]);
}

void killLEDs(){	// kills all LEDs
	sendMessage(GP0, 0x00, 0x00);
}

void sendMessage(int reg, int message){
	Wire.beginTransmission(0x20);
	Wire.write(reg);
	Wire.write(message);
	Wire.endTransmission();
}

void sendMessage(int startReg, int message1, int message2){
	Wire.beginTransmission(0x20);
	Wire.write(startReg);
	Wire.write(message1);
	Wire.write(message2);
	Wire.endTransmission();
}

void reInit(){
	print("Reinitialising state booleans..",0);
	
	started = false;
	fridgeUnlocked = false;
	for(int i=0;i<8;i++){
		touchedNodes[i] = false;
	}	
	for(int i=0;i<5;i++){
		stepsCompleted[i] = false;
	}
	digitalWrite(unlockedLed, LOW);
	cycleCount = 0;
	print(" ..done\n",0);
}

int print(String message, int type){	// wrapper for Serial.print to print based on debug state
	if((type==0)&&DEBUG){
		Serial.print(message);
		return type;
	}else if((type==1)&&FDEBUG){
		Serial.print(message);
		return type;
	}else return -1;	
}

int print(long message, int type){
	return print((String)message,type);
};