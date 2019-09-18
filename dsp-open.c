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
#include "fxlms.c"


#define NUM_ALL_INPUTS		40
#define NUM_OUTPUTS		8
#define NUM_INPUTS 		8

#define DAC_MAX_OUTPUT 0.65
#define DAC_GAIN (30.0/32*10)
#define ADC_FILTER_GAIN (30.0/32)


static char dsp_device[40]; 

#define REFERENCE_N		512
#define SEC_FILTER_N		128
#define ADAPTATION_FILTER_N 	128
#define FEEDBACK_N		64
#define CONTROL_N		5
#define REFERENCE_CHANNEL 	0

const float ****s;
struct lf_ring ***r;
struct lf_ring *e;
static struct lf_ring x[ADAPTATION_FILTER_N * 2];
static struct lf_ring feedback_ref[8];

float w[CONTROL_N][CONTROL_CHANNELS][ADAPTATION_FILTER_N];

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

static int return_greater(int current, int next){
	if(current > next)
		return current;
	else return next;
}

static int get_dsp_seq(){

	int dsp_seq = 0; // FIXME

	for (int i = 0; i < CONTROL_N; i++){
		snprintf(dsp_device, sizeof(dsp_device), "/dev/ds1104-%d-mem", i);
		if(dsp_remap(DSP_RING_BASE, dsp_device))
			fprintf(stderr, "cannot intialize DSP communication");
	//	printf("seq: %d\n", dsp_seqnum());
		dsp_seq = return_greater(dsp_seq, dsp_seqnum());		
	}
	return dsp_seq;
}

static void send_data(){
	//int16_t *dst_test;
	//dst_test = malloc(ADAPTATION_FILTER_N*4*2);
	int32_t *w_to_dst;
	w_to_dst = malloc(ADAPTATION_FILTER_N*4*sizeof(*w_to_dst));

	for(int num_chan = 0; num_chan < CONTROL_N; num_chan++){
		int next = 0;

		for(int wui = 0; wui < plate_params.u; wui ++){
			float sum = 0;
			float sum2 = 0;
				for(int wi = 0; wi < plate_params.n; wi ++){
					sum += w[num_chan][wui][wi];
					sum2 += w[num_chan][wui][wi] * w[num_chan][wui][wi];
					//w[num_chan][wui][wi] = wi/2.0;
					w_to_dst[next] = __builtin_bswap32(*((int32_t *)&w[num_chan][wui][wi]));
					next ++;
				}
			fprintf(stderr, "ch%d-%d: %f %f\n", num_chan, wui, sum, sum2);
		}
		snprintf(dsp_device, sizeof(dsp_device), "/dev/ds1104-%d-mem", 0);
		send_to_dsp(w_to_dst, dsp_device, ADAPTATION_FILTER_N, plate_params.u );
	}
}

static void adaptation(double mi, int num_channel){
		complex alfa[ADAPTATION_FILTER_N * 2], e_com[ADAPTATION_FILTER_N * 2], r_com[ADAPTATION_FILTER_N * 2];

		unsigned int xi = 0;
		do
		{
			unsigned int ei = 0;
			do
			{
				prepare_err(e_com, &e[ei]);
				fft(e_com, ADAPTATION_FILTER_N * 2);
				unsigned int ui = 0;
				do
				{
					prepare_ref(r_com, &r[xi][ei][ui]);
					fft(r_com, ADAPTATION_FILTER_N * 2);
					calculate_alfa(e_com, r_com, alfa);
					ifft(alfa, ADAPTATION_FILTER_N * 2);
					for(int i = 0; i < ADAPTATION_FILTER_N; i++){

						w[num_channel][ui][i] = w[num_channel][ui][i] - mi * creal(alfa[i]);
		//			printf("%f ", w[num_channel][ui][i]);
					}
			//		printf("\n");
					ui++;
				} while (ui < plate_params.u);
			} while (++ei < plate_params.e);
		xi++;
		} while (xi < plate_params.x);
}

int main() {

	float outputs[ADAPTATION_FILTER_N * 2][CONTROL_N][NUM_OUTPUTS];
	float inputs[ADAPTATION_FILTER_N * 2][NUM_ALL_INPUTS];
	float adc_scaler=1.0 / (32678.0 * ADC_FILTER_GAIN);
	

	init_models();

	fxlms_initialize_s();

	for (int num_channel = 0; num_channel < CONTROL_N; num_channel++){
		for(int ei = 0; ei < plate_params.e; ei++){
			for(int ui = 0; ui < plate_params.u; ui++){
				s[num_channel][ei][ui] = models[num_channel][ui][num_errors[ei]];
			}
		}
	}    

	for(;;){
		int16_t *dst; 

		int dsp_seq = 0;
		double mi;

		fxlms_initialize_e();

		dsp_seq = get_dsp_seq();

		for(int i=0; i < CONTROL_N; i++){
			snprintf(dsp_device, sizeof(dsp_device), "/dev/ds1104-%d-mem", i);
			if(dsp_remap(DSP_RING_BASE, dsp_device))
				fprintf(stderr, "cannot intialize DSP communication");

			dst = malloc(DSP_PACKAGE_SIZE);
			copy_from_dsp(dst,dsp_device, dsp_seq - 2);
			for(int in = 0; in < ADAPTATION_FILTER_N * 2; in++){

				for(int j = 0; j < NUM_OUTPUTS; j++)
							outputs[in][i][j] = dst[(NUM_INPUTS + NUM_OUTPUTS)*in + NUM_INPUTS + j]/(DAC_MAX_OUTPUT/DAC_GAIN*32768.0);
				
				for(int j = 0; j < NUM_INPUTS; j++)
					inputs[in][i * NUM_INPUTS + j] = dst[(NUM_INPUTS + NUM_OUTPUTS)*in + j] * adc_scaler;
					
				}
			free(dst);
		}

		double x_full[ADAPTATION_FILTER_N * 2];

		for(int in = 0; in < ADAPTATION_FILTER_N * 2; in++){

			for(int j=0; j < CONTROL_N; j++){

				x_full[in] =  inputs[in][REFERENCE_CHANNEL];
	
#if 0
				for (int i = 0; i < 8; i++) {
					lf_ring_init(feedback_ref + i, ADAPTATION_FILTER_N * 2);
					lf_ring_set_buffer(feedback_ref + i, NULL, 0);
				}

				for (int ui = 0; ui < plate_params.u; ui++)
					lf_ring_add(&feedback_ref[ui], outputs[in][j][ui]);
			
				x_full[in] -= feedback_neutralization(j);
#endif
			}
		}

	for (int num_channel = 0; num_channel < CONTROL_N; num_channel++){

		fxlms_initialize_r();

		for(int sample = 0; sample < ADAPTATION_FILTER_N * 2; sample++){
			lf_ring_init(x + sample, ADAPTATION_FILTER_N * 2);
			lf_ring_set_buffer(x + sample, NULL, 0);
		}

		for(int sample=0; sample < ADAPTATION_FILTER_N * 2; sample++){
			lf_ring_add(&x[sample], x_full[sample]);
			
			for(int ei = 0; ei < plate_params.e; ei++ ){
				lf_ring_add(&e[ei], inputs[sample][num_errors[ei]]);
			}
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

		mi = fxlms_normalize(r);
	//	printf("norm: %1.25lf\n", mi);

		adaptation(mi, num_channel);
	}

	send_data();

	}

return 0;
}
 
//uint16_t x = readl(shm_save);
//printf("TEST ILOSC: %d", x);
//uint16_t y = readl(shm_save + 0x04);
//printf("TEST STEROWANIA: %d", y);
//memcpy_fromio(dst_test, shm_save, 128 * 4 * 2);
//for(int k = 0; k < 128 * 4; k++){
//	printf("%f ", dst_test[k]/327680.0);
//}
