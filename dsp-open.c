#include "asm_io.h"
#include "compiler.h"
#include "pu-shm.h"
#include "string.h"
#include <stdio.h>
#include <sys/stat.h> 
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdint.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>
#include "models-init.h"
#include "ring.c"
#include "fir.c"
#include "fft.c"
#include "pu-shm.c"


//#define DSP_RING_ENTRIES 0x18
//#define DSP_ALIGMENT 0x1000
//#define DSP_SAMPLE_SIZE 0x20
//#define DSP_SEQNUM 0x08
//#define DSP_RING_BASE 0x1000
//#define DSP_PACKAGE_SIZE 0x2000
#define NUM_ALL_INPUTS		40
#define NUM_OUTPUTS		8
#define NUM_INPUTS 		8

//#define DSP_WEIGHT_PACKAGE_SIZE 128*4*2
//#define DSP_WEIGHT_OFFSET 0X500
#define DAC_MAX_OUTPUT 0.65
#define DAC_GAIN (30.0/32*10)
#define ADC_FILTER_GAIN (30.0/32)


static const char *dsp_device[40]; 

//static unsigned int ring_entries;

struct fxlms_params
{
	double mu;
	double zeta;
	unsigned int u;
	unsigned int e;
	unsigned int x;
	unsigned int n;
	unsigned int m;
};


#define REFERENCE_N		512
#define SEC_FILTER_N		128
#define ADAPTATION_FILTER_N 	128
#define FEEDBACK_N		64
#define CONTROL_N		5	
#define REFERENCE_CHANNEL 	0

const float ****s;
struct lf_ring ***r;
struct lf_ring *e;
static struct lf_ring x[256];
static struct lf_ring feedback_ref[8];

static struct fxlms_params plate_params = {
	.u	= 4,
	.e	= 9,
	.x	= 1,
	.m	= 128,
	.n	= 128,
	.mu	= 0.001,
	.zeta	= 1e-6,
};

static int num_errors[9] = {1,2,3,9, 10, 11,19, 27, 35};

static double feedback_neutralization(int node_id)
{
	unsigned int ui;
	double f = 0;

	for (ui = 0; ui < plate_params.u ; ui++)
		f = lf_fir(&feedback_ref[ui], models[node_id][ui][REFERENCE_CHANNEL], FEEDBACK_N);
	return f;
}

static void fxlms_initialize_r()
{
	r = malloc(plate_params.x * sizeof(*r));
	for (int xi = 0; xi < plate_params.x; xi++) {
		r[xi] = malloc(plate_params.e * sizeof(**r));
		for (int ei = 0; ei < plate_params.e; ei++) {
			r[xi][ei] = malloc(plate_params.u * sizeof(***r));
			for (int ui = 0; ui < plate_params.u; ui++) {
				lf_ring_init(&r[xi][ei][ui], SEC_FILTER_N);
				lf_ring_set_buffer(&r[xi][ei][ui], NULL, 0);
			}
		}
	}
}

static void fxlms_initialize_s()
{
	s = malloc(CONTROL_N * sizeof(*s));
	for(int num_channels = 0; num_channels < CONTROL_N; num_channels++){
	s[num_channels] = malloc(plate_params.e * sizeof(**s));
	for (int ei = 0; ei < plate_params.e; ei++)
		s[num_channels][ei] = malloc(plate_params.u * sizeof(***s));
	}
}	

static void fxlms_initialize_e(){
	e = malloc(plate_params.e * sizeof(*e));
	for (int ei = 0; ei < plate_params.e; ei++) {
		lf_ring_init(&e[ei], SEC_FILTER_N);
		lf_ring_set_buffer(&e[ei], NULL, 0);
	}
}


static double fxlms_normalize(const struct lf_ring ***r){

	unsigned int ui = 0;
	unsigned int xi = 0;	
	float power = 0;

	unsigned int ei = 0;

	for(int i = 0; i<256; i++){
		xi = 0;
		do{
			ei = 0;
			do{
				ui = 0;
				do{

				float t = lf_ring_get(&r[xi][ei][ui], i);
				power += t * t;

				} while(++ui < plate_params.u);
			} while(++ei < plate_params.e);
		} while(++xi < plate_params.x);
	}
	return plate_params.mu / (power + plate_params.zeta);
}

static int return_greater(int current, int next){
	if(current > next)
		return current;
	else return next;
}

void prepare_ref(complex *r_com, const struct lf_ring *r){
	for (int i = 0; i < ADAPTATION_FILTER_N * 2; i++){
		r_com[i]  = lf_ring_get(r, i);
	}
}

void prepare_err(complex *e_com, const struct lf_ring *e){
	for (int i = 0; i < ADAPTATION_FILTER_N; i++){
		e_com[i]  = lf_ring_get(e, (ADAPTATION_FILTER_N * 2) - i); 
		e_com[ADAPTATION_FILTER_N + i] = 0.0;
	}
}

complex calculate_alfa(complex *e_com, complex *r_com, complex *alfa){
	for(int i = 0; i < ADAPTATION_FILTER_N * 2; i++){
		alfa[i] = conj(r_com[i]) * e_com[i];
	}
}

int main() {

	//load_data_from_memory from all cards
	float outputs[ADAPTATION_FILTER_N * 2][CONTROL_N][NUM_OUTPUTS];
	float inputs[ADAPTATION_FILTER_N * 2][NUM_ALL_INPUTS];
	float adc_scaler=1.0 / (32678.0 * ADC_FILTER_GAIN);
	
	float w[CONTROL_N][plate_params.u][ADAPTATION_FILTER_N];

	init_models();

	for(int z = 0; z< 1; z++){
	int16_t *dst; 

	int dsp_seq = 0;

	for (int i = 0; i < CONTROL_N; i++){

		snprintf(dsp_device, sizeof(dsp_device), "/dev/ds1104-%d-mem", i);
		if(dsp_remap(DSP_RING_BASE, dsp_device));
			printf(stderr, "cannot intialize DSP communication");
		printf("seq: %d\n", dsp_seqnum());
		dsp_seq = return_greater(dsp_seq, dsp_seqnum());		
	//	ring_entries = readl(shm + DSP_RING_ENTRIES);
	}
	for(int i=0; i < CONTROL_N; i++)
	{
		snprintf(dsp_device, sizeof(dsp_device), "/dev/ds1104-%d-mem", i);
		if(dsp_remap(DSP_RING_BASE, dsp_device));
			printf(stderr, "cannot intialize DSP communication");

		dst = malloc(DSP_PACKAGE_SIZE);
		copy_from_dsp(dst,dsp_device, dsp_seq - 2);
		for(int in = 0; in < ADAPTATION_FILTER_N * 2; in++){

			for(int j = 0; j < NUM_OUTPUTS; j++)
		     			outputs[in][i][j] = dst[16*in + NUM_INPUTS + j]/(DAC_MAX_OUTPUT/DAC_GAIN*32768.0);
			
			for(int j = 0; j < NUM_INPUTS; j++)
				inputs[in][i * NUM_INPUTS + j] = dst[16*in + j] * adc_scaler;
				
			}
	}

	double x_full[ADAPTATION_FILTER_N * 2];

	for(int in = 0; in < ADAPTATION_FILTER_N * 2; in++){

		for(int j=0; j < CONTROL_N; j++){

			x_full[in] =  inputs[in][REFERENCE_CHANNEL];
	
			for (int i = 0; i < 8; i++) {
				lf_ring_init(feedback_ref + i, ADAPTATION_FILTER_N * 2);
				lf_ring_set_buffer(feedback_ref + i, NULL, 0);
			}

			for (int ui = 0; ui < plate_params.u; ui++)
				lf_ring_add(&feedback_ref[ui], outputs[in][j][ui]);
		
			x_full[in] -= feedback_neutralization(j);
		}
	}

double mu[CONTROL_N];

	fxlms_initialize_e();
	fxlms_initialize_s();

for (int num_channel = 0; num_channel < CONTROL_N; num_channel++){

	//alokacja miejsca na model s path
        fxlms_initialize_r();

	//set s from models
	for(int ei = 0; ei < plate_params.e; ei++){
		for(int ui = 0; ui < plate_params.u; ui++){
			s[num_channel][ei][ui] = models[num_channel][ui][num_errors[ei]];
		}
	}

	//init x to lfir
	for(int sample = 0; sample < ADAPTATION_FILTER_N * 2; sample++){
		lf_ring_init(x + sample, ADAPTATION_FILTER_N * 2);
		lf_ring_set_buffer(x + sample, NULL, 0);
	}

	for(int sample=0; sample < ADAPTATION_FILTER_N * 2; sample++){
		lf_ring_add(&x[sample], x_full[sample]);
		
		for(int ei = 0; ei < plate_params.e; ei++ ){
			lf_ring_add(&e[ei], inputs[sample][num_errors[ei]]);
		}

		//filter reference signal with s
		int xi = 0;
		do {
			int ei = 0;
			do {
				int ui = 0;
				do {
					double r_temp;

					r_temp = lf_fir(&x[sample], s[num_channel][ei][ui], SEC_FILTER_N);
					lf_ring_add(&r[xi][ei][ui], r_temp);
					ui++;
				} while (ui < plate_params.u);
			} while (++ei < plate_params.e);
			xi++;
		} while (xi < plate_params.x);
	}


	mu[num_channel] = fxlms_normalize(r);
	printf("norm: %1.25lf\n",mu[num_channel] );

	complex alfa[ADAPTATION_FILTER_N * 2], e_com[ADAPTATION_FILTER_N * 2], r_com[ADAPTATION_FILTER_N * 2];

	unsigned int xxi = 0;
	do
	{
		unsigned int eei = 0;
		do
		{
			prepare_err(e_com,&e[eei]);
			fft(e_com, ADAPTATION_FILTER_N * 2);
			unsigned int uui = 0;
			do
			{
				prepare_ref(r_com, &r[xxi][eei][uui]);
				fft(r_com, ADAPTATION_FILTER_N * 2);
				calculate_alfa(e_com, r_com, alfa);
				ifft(alfa, ADAPTATION_FILTER_N * 2);
				for(int i = 0; i < ADAPTATION_FILTER_N; i++){

					w[num_channel][uui][i] = w[num_channel][uui][i] - mu[num_channel] * creal(alfa[i]);
		//			printf("%f ", w[num_channel][uui][i]);
				}
		//		printf("\n");
				uui++;
			} while (uui < plate_params.u);
		} while (++eei < plate_params.e);
	xxi++;
	} while (xxi < plate_params.x);
}

/**************************************************************************************************************************************/


/*******************************************WRITE DATA - SHARED MEMORY ********************************************************/


int16_t *dst_test;
dst_test = malloc(ADAPTATION_FILTER_N*4*2);
int16_t *w_to_dst;
w_to_dst = malloc(ADAPTATION_FILTER_N*4*2);



for(int num_chan = 0; num_chan < CONTROL_N; num_chan++){
	int next = 0;
	for(int wui = 0; wui < plate_params.u; wui ++){
			for(int wi = 0; wi < plate_params.n; wi ++){
				w_to_dst[next] = w[num_chan][wui][wi] * 327680;
				//printf("%d ", w_to_dst[next]);
				next ++;
			}
	}
	snprintf(dsp_device, sizeof(dsp_device), "/dev/ds1104-%d-mem", 0);
	send_to_dsp(w_to_dst, dsp_device, ADAPTATION_FILTER_N, plate_params.u );
}

//uint16_t x = readl(shm_save);

//printf("TEST ILOSC: %d", x);

//uint16_t y = readl(shm_save + 0x04);

//printf("TEST STEROWANIA: %d", y);

//memcpy_fromio(dst_test, shm_save, 128 * 4 * 2);

//for(int k = 0; k < 128 * 4; k++){
//	printf("%f ", dst_test[k]/327680.0);
//}

}

/**************************************************************************************************************************************/

return 0;
}
 
