/*This is teh code for a Spectrum Analyser

Parts of the code were suppressed for clarity since they work with Adafruti Neopixel

Code was borrowed from many different sources
Among them:
* Light spectrum - FFT by Mike Cook
* for Arduino Due & Neopixel strip
* Each bin covers 21.701 Hz
*/

#include "SplitRadixRealP.h"					//Using local versions due to some slight modifications
#include "filters.h"
#include <FastLED.h>

const float cutoff_freq   = 315.0;  			//Cutoff frequency in Hz
const float sampling_time = 0.00002273; 		//Sampling time in seconds.
IIR::ORDER  order  = IIR::ORDER::OD3; 			// Order (OD1 to OD4)
Filter f(cutoff_freq, sampling_time, order);	// Low-pass filter

#define NUM_LEDS_PER_STRIP 256
#define NUM_STRIPS 2
CRGB leds[NUM_STRIPS * NUM_LEDS_PER_STRIP];

#define xMax 32                   				// maximum number of coloumns
#define yMax 8                    				// maximum number of rows

#define SMP_RATE      44444UL
#define CLK_MAIN      84000000UL
#define TMR_CNTR      CLK_MAIN / (2 *SMP_RATE)

#define   FFT_SIZE      2048      				//For f>315Hz, sampling at 44444 and 21.701Hz per bin
#define   FFT_SIZE_512  512       				//For f<315Hz, subsampling from above at 2020 and 3.9457Hz per bin
#define   MIRROR      FFT_SIZE / 2  			//Mirror image of bins for FFT_2048
#define   MIRROR_512  FFT_SIZE_512 / 2  		//Mirror image of bins for FFT_512
#define   INP_BUFF    FFT_SIZE      			//Input buffer for samples
#define   INP_BUFF512 FFT_SIZE_512				//Second buffer derived from above

volatile uint16_t sptr = 0 ;
volatile int16_t flag = 0 ;
uint16_t inp[2][INP_BUFF] = { 0}; 				//DMA likes ping-pongs buffer
int f_r[FFT_SIZE] = { 0};
int f_r_512[FFT_SIZE_512] = {0};
int out1[MIRROR] = { 0};          				//Bin magnitudes using get_Magnit1, FFT_2048
int out2[MIRROR] = { 0};          				//Bin magnitudes using get_Magnit2. FT_2048
int out1_512[MIRROR_512] = { 0};  				//Bin magnitudes using get_Magnit1, FFT_512
int out2_512[MIRROR_512] = { 0};  				//Bin magnitudes using get_Magnit2, FFT_512
const int dc_offset = 2047;       				//Virtual ground offset on ADC input
int LOG2_FFT = 11;                				//Log2 of FFT_2048 size
int LOG2_FFT_512 = 9;							//Log2 of FFT_512 size
SplitRadixRealP radix;							//Creates instance of SplitRadix

//Bin allocation into frequencies
const int binSplit[] ={0,0,0,0,0,1,2,3,4,5,6,8,10,14,17,21,26,33,41,52,65,82,103,130,164,207,261,328,414,521,657,828,1024};                     //For FFT 44444/2048
const int binSplit_512[] ={3,4,6,7,9,11,14,18,22,28,36,45,56,71,90,113,142,179,226,255,255,255,255,255,255,255,255,255,255,255,255,255,255};  	//For FFT 2020/512

//Bins for holding each column of the strip (each frequency)
int binTotal[xMax];
int binTotal_512[xMax];

//Lin to log table
int LinToLog[]={8,16,32,65,129,257,513,725};   	//For 1024 ADC readout, 6dB per LED (8 LEDS, to be modified for final version with 16 LEDs)

void pio_TIOA0 () { 							// Configure Ard pin 2 as output from TC0 channel A (copy of trigger event)
	PIOB->PIO_PDR = PIO_PB25B_TIOA0 ; // disable PIO control
	PIOB->PIO_IDR = PIO_PB25B_TIOA0 ; // disable PIO interrupts
	PIOB->PIO_ABSR |= PIO_PB25B_TIOA0 ; // switch to B peripheral
}

void timer_setup (){
	pmc_enable_periph_clk(TC_INTERFACE_ID + 0 *3 + 0); // clock the TC0 channel 0
	TcChannel * t = &(TC0->TC_CHANNEL)[0] ; // pointer to TC0 registers for its channel 0
	t->TC_CCR = TC_CCR_CLKDIS ; // disable internal clocking while setup regs
	t->TC_IDR = 0xFFFFFFFF ; // disable interrupts
	t->TC_SR ; // read int status reg to clear pending
	t->TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK1 | // use TCLK1 (prescale by 2, = 42MHz)
	TC_CMR_WAVE | // waveform mode
	TC_CMR_WAVSEL_UP_RC | // count-up PWM using RC as threshold
	TC_CMR_EEVT_XC0 | // Set external events from XC0 (this setup TIOB as output)
	TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_CLEAR |
	TC_CMR_BCPB_CLEAR | TC_CMR_BCPC_CLEAR ;
	t->TC_RC = TMR_CNTR; // counter resets on RC, so sets period in terms of 42MHz clock
	t->TC_RA = TMR_CNTR /2; // roughly square wave
	t->TC_CMR = (t->TC_CMR & 0xFFF0FFFF) | TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_SET ; // set clear and set from RA and RC compares
	t->TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG ; // re-enable local clocking and switch to hardware trigger source.
}

void adc_setup (){
	pmc_enable_periph_clk(ID_ADC);
	adc_init(ADC, SystemCoreClock, ADC_FREQ_MAX, ADC_STARTUP_FAST);
	NVIC_EnableIRQ (ADC_IRQn); // enable ADC interrupt vector
	adc_disable_all_channel(ADC);
	adc_enable_interrupt(ADC, ADC_IER_RXBUFF);
	ADC->ADC_RPR = (uint32_t) inp[0]; // DMA buffer
	ADC->ADC_RCR = INP_BUFF;
	ADC->ADC_RNPR = (uint32_t) inp[1]; // next DMA buffer
	ADC->ADC_RNCR = INP_BUFF;
	ADC->ADC_PTCR = 1;
	adc_set_resolution (ADC, ADC_12_BITS);
	ADC-> ADC_CGR = 0x00000003;    //Ganho =4 para ADC_CHANNEL_7, AN0
	ADC-> ADC_COR = 0x00000001;    //Offset compensado, AN0
	adc_set_bias_current(ADC, 0x01);
	adc_enable_channel(ADC, ADC_CHANNEL_7); // AN0
	adc_configure_trigger(ADC, ADC_TRIG_TIO_CH_0, 0);
	adc_start(ADC);
}

void ADC_Handler (void){
	if((adc_get_status(ADC) & ADC_ISR_RXBUFF) == ADC_ISR_RXBUFF){
	flag = ++sptr;
	sptr &= 0x01; // alternate buffer to use
	ADC->ADC_RNPR = (uint32_t) inp[sptr];
	ADC->ADC_RNCR = INP_BUFF;
  }
}

void setup(){
	Serial.begin (115200); // fast baud
	FastLED.addLeds<WS2811_PORTD,NUM_STRIPS>(leds, NUM_LEDS_PER_STRIP);
	adc_setup ();
	timer_setup ();
	pinMode( 25, OUTPUT);   //Port D0
	pinMode( 26, OUTPUT);   //Port D1
 }

inline int mult_shft12( int a, int b){
 return (( a * b ) >> 12);
}

void loop(){
	if (flag){
	uint16_t indx_a = flag -1;
	uint16_t indx_b = 0;
	//For FFT_2048 
	for ( uint16_t i = 0, k = (NWAVE / FFT_SIZE); i < FFT_SIZE; i++ ){
	uint16_t windw = Hamming[i * k];
	f_r[i] = mult_shft12((inp[indx_a][indx_b++] - dc_offset), windw);			//Hamming windows on samples
	}
	radix.rev_bin (f_r, FFT_SIZE);												//FFT_2048
	radix.fft_split_radix_real (f_r, LOG2_FFT);
	radix.gain_Reset (f_r, LOG2_FFT -1, FFT_SIZE);
	radix.get_Magnit2 (f_r, out2, MIRROR, FFT_SIZE);
	// Magnitudes:-
	bufferToBinTotal();															//Converts FFT bins to frequency bins

	//For FFT_512
	for (uint16_t i = 1; i < 419; i++){   //Shifts samples from circular buffer for calculating FFT_512 togheter with FFT_2048. keeping the same frame rate
		inp_512[i]=inp_512[i+93];
	}

	//3rd order LPF dor eliminating frequencies above 2020/2 Hz
	for (uint16_t i = 0; i<FFT_SIZE; i++){
		int value = inp[indx_a][i];
		inp[indx_a][i] = f.filterIn(value);
    }
    
	indx_b = 0;
	for (uint16_t j = 419; j < FFT_SIZE_512; j++){
		inp_512[j]=inp[indx_a][indx_b*22];
		indx_b++;
	}
	indx_b=0;
	for ( uint16_t i = 0, k = (NWAVE / FFT_SIZE_512); i < FFT_SIZE_512; i++ ){
		uint16_t windw = Hamming[i * k];
		f_r_512[i] = mult_shft12((inp_512[indx_b++] - dc_offset), windw);
	}
	radix.rev_bin (f_r_512, FFT_SIZE_512);									//FFT_512
	radix.fft_split_radix_real (f_r_512, LOG2_FFT_512);
	radix.gain_Reset (f_r_512, LOG2_FFT_512 -1, FFT_SIZE_512);
    radix.get_Magnit2 (f_r_512, out2_512, MIRROR_512, FFT_SIZE_512);
	// Magnitudes:-
	bufferToBinTotal_512();
	
	binsToLEDs();															//Converts frequency bins to columns of LEDS, using logathimic table
	FastLED.show();															//Update LEDs
	flag = 0;
	}
}

void bufferToBinTotal(){
	uint16_t number = 0;
	for(int i=0; i< xMax; i++){
		number = 0;
		binTotal[i] = 0;
		for(int k = binSplit[i]; k < binSplit[i+1]; k++){
			binTotal[i] += out2[k];
			number++;
		}
	}
}

void bufferToBinTotal_512(){     
	uint16_t number = 0;
	for(int i=0; i< xMax; i++){
		number = 0;
		binTotal_512[i] = 0;
		for(int k = binSplit_512[i]; k < binSplit_512[i+1]; k++){
			binTotal_512[i] += out2_512[k];
			number++;
		}
	}
 }

void binsToLEDs(){
 int aux = 0;

 // 32 strips com 8 LEDs cada, valor de setStrip (número da strip, LEDs acesos - de 1 a 8)
 //Achar maneira mais otimizada de fazer a conta e deixar independente do número de linhas por coluna
  
  for(int i=0; i< xMax; i++){     //setStrip vai de 1 ao número de LEDs por coluna
                                  //Bins 0-12 pegar do binTotal_512, bins 13-31 pegar do binTotal
    if (i<13) {  //valor certo = 13
    aux=0;
    if (binTotal_512[i] >= LinToLog[0]) aux=1;
    if (binTotal_512[i] >= LinToLog[1]) aux=2;
    if (binTotal_512[i] >= LinToLog[2]) aux=3;
    if (binTotal_512[i] >= LinToLog[3]) aux=4;
    if (binTotal_512[i] >= LinToLog[4]) aux=5;
    if (binTotal_512[i] >= LinToLog[5]) aux=6;
    if (binTotal_512[i] >= LinToLog[6]) aux=7;
    if (binTotal_512[i] >= LinToLog[7]) aux=8;
    setStrip(i,aux);  
    }
    else {      
    aux=0;
    if (binTotal[i] >= LinToLog[0]) aux=1;
    if (binTotal[i] >= LinToLog[1]) aux=2;
    if (binTotal[i] >= LinToLog[2]) aux=3;
    if (binTotal[i] >= LinToLog[3]) aux=4;
    if (binTotal[i] >= LinToLog[4]) aux=5;
    if (binTotal[i] >= LinToLog[5]) aux=6;
    if (binTotal[i] >= LinToLog[6]) aux=7;
    if (binTotal[i] >= LinToLog[7]) aux=8;
    setStrip(i,aux);
    }
  }
}

void setStrip(int strip, int height){
	int stripColour;
    if(height > yMax) height = yMax;
    for(int y=0; y < height; y++){
    stripColour = colour(3,1,3);
    leds[getPixPos(strip,y)] = stripColour;
    }
}

int getPixPos(int x, int y){ // for a serpentine raster
	int pos;
	if(x &0x1) {
		pos = x * yMax + y ;
	}
	else {
		pos = x * yMax + (yMax -1 -y);
	}
	return pos;
}

inline int colour( int a, int b, int c){
 return ((a << 16) | (b << 8) | c);
}
