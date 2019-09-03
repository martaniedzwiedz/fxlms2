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


#define DSP_RING_ENTRIES 0x18
#define DSP_ALIGMENT 0x1000
#define DSP_SAMPLE_SIZE 0x20
#define DSP_SEQNUM 0x08
#define DSP_RING_BASE 0x1000
#define DSP_PACKAGE_SIZE 0x2000
#define NUM_ALL_INPUTS		40
#define NUM_OUTPUTS		8
#define NUM_INPUTS 		8

#define DSP_WEIGHT_PACKAGE_SIZE 0X100
#define DSP_WEIGHT_OFFSET 0X500

unsigned long long shm_offset = 0x1000000;

static const char *dsp_device[40]; // = "/dev/ds1104-0-mem";
static const uint8_t *shm, *shm_save;

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

unsigned int *dsp_get_buffer(unsigned int seq)
{
	unsigned long offset;
	offset = seq * DSP_ALIGMENT;
	return shm + DSP_RING_BASE + offset;
}

void __copy_from_dsp(unsigned int seq, void *dst)
{
	const uint8_t *buffer = dsp_get_buffer(seq);
	memcpy_fromio(dst, buffer, DSP_PACKAGE_SIZE);
}

static unsigned int dsp_seqnum()
{
	uint32_t x;

	x = readl(shm + DSP_SEQNUM);
	return x;
}

static int dsp_remap(unsigned long size, char *dsp_device)
{
	void *p;
	int ret;
	int fd; 

	fd = open(dsp_device, O_RDONLY);
	if(fd == -1){
		perror("cannot open DSP device");
		ret = -errno;
		goto out;
	}

	p = mmap(NULL, size, PROT_READ, MAP_SHARED, fd, shm_offset);

	if(p==MAP_FAILED){
		perror("cannot map DSP shared memory");
		ret = -errno;
		goto out_close;
	
	}

	shm = p;
	ret = 0;
out_close:
	close(fd);
out: 
	return ret;
}

void show_data(int16_t *dst, unsigned int seq)
{

	for(int i=0; i<128;i++)
	{
		printf("%d: ", seq);
			for(int j=0; j<16 ; j++)
			{	
				double x = dst[16*i+j]/32768.0;
				printf("%f ",x);
			}
		printf("\n");
	}
}

static int dsp_begin(char *dsp_device)
{

	int ret;
	uint32_t x;

	x = readl(shm + DSP_SHM_SIZE);
	if(x < DSP_RING_BASE){
		fprintf(stderr, "too small shared memory region: %d KiB\n", (unsigned int)x);
		return -EINVAL;
	}

	ret = dsp_remap(x, dsp_device);
	if(ret)
		return ret;

	return 0;
}

void copy_from_dsp(void * dst, char *dsp_device, unsigned int dsp_seq)
{
	//unsigned int dsp_seq;
	int ret;

	ret = dsp_begin(dsp_device);
	//dsp_seq = dsp_seqnum();
	__copy_from_dsp(dsp_seq, dst );
	show_data(dst, dsp_seq);
}

void send_to_dsp(int8_t *dst, char *dsp_device)
{
	
	void *filty;
	int ret;
	int fd; 

	fd = open(dsp_device, O_RDWR);
	if(fd == -1){
		perror("cannot open DSP device");
	}

	filty = mmap(NULL, 0xf0000, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0xf00000);

	if(filty==MAP_FAILED){
		perror("cannot map DSP shared memory");
	}

	shm_save = filty;

	memcpy_toio(filty, dst, DSP_WEIGHT_PACKAGE_SIZE);
}

static unsigned int error_mic_to_input(unsigned int p)
{
	return (p << 3) + 3;
}
#define REFERENCE_N		512
#define SEC_FILTER_N		128
#define ADAPTATION_FILTER_N 	256
#define FEEDBACK_N		64
#define CONTROL_N		5
#define REFERENCE_CHANNEL 	0

const float ****s;
struct lf_ring ****r;
struct lf_ring *e;
static struct lf_ring x;
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
	r = malloc(CONTROL_N * sizeof(*r));
	for(int num_channels = 0; num_channels < CONTROL_N; num_channels++){
		r[num_channels] = malloc(plate_params.x * sizeof(**r));
		for (int xi = 0; xi < plate_params.x; xi++) {
			r[xi] = malloc(plate_params.e * sizeof(***r));
			for (int ei = 0; ei < plate_params.e; ei++) {
				r[xi][ei] = malloc(plate_params.u * sizeof(****r));
				for (int ui = 0; ui < plate_params.u; ui++) {
					lf_ring_init(&r[xi][ei][ui], SEC_FILTER_N);
					lf_ring_set_buffer(&r[xi][ei][ui], NULL, 0);
				}
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
		s[ei] = malloc(plate_params.u * sizeof(***s));
	}
}	

static double fxlms_normalize(const struct lf_ring ***r){
	
	unsigned int ui = 0;
	unsigned int xi = 0;	
	unsigned int i = 0;
	double power = 0;

	unsigned int ei = 0;

	ei = 0;
	do{
		xi = 0;
		do{
			ui = 0;
			do{
				i = 0;
				do{
					float t = lf_ring_get(&r[xi][ei][ui], i);
					power += t * t;
			       		i++;	
				}while(i < plate_params.n);
			}while(++ui < plate_params.u);
		}while(++xi < plate_params.x);
	}while(++ei < 2);
	return plate_params.mu / (power + plate_params.zeta);
};	

static int return_greater(int current, int next){
	if(current > next)
		return current;
	else return next;
}

int main() {

	//load_data_from_memory from all cards
	float outputs[ADAPTATION_FILTER_N][CONTROL_N][NUM_OUTPUTS];
	float inputs[ADAPTATION_FILTER_N][NUM_ALL_INPUTS];
	
	int16_t *dst; 

	int dsp_seq = 0;

	for (int i = 0; i < CONTROL_N; i++){

		snprintf(dsp_device, sizeof(dsp_device), "/dev/ds1104-%d-mem", i);
		if(dsp_remap(DSP_RING_BASE, dsp_device));
			printf(stderr, "cannot intialize DSP communication");
		printf("seq: %d\n", dsp_seqnum());
		dsp_seq = return_greater(dsp_seq, dsp_seqnum());

	}
	for(int i=0; i < CONTROL_N; i++)
	{
		snprintf(dsp_device, sizeof(dsp_device), "/dev/ds1104-%d-mem", i);
		if(dsp_remap(DSP_RING_BASE, dsp_device));
			printf(stderr, "cannot intialize DSP communication");

		dst = malloc(DSP_PACKAGE_SIZE);
		copy_from_dsp(dst,dsp_device, dsp_seq - 2);
		for(int in = 0; in < ADAPTATION_FILTER_N; in++){
			for(int j = 0; j < NUM_OUTPUTS; j++)
		     		outputs[in][i][j] = dst[DSP_SAMPLE_SIZE*in + NUM_INPUTS + j]/32768.0;	
			for(int j = 0; j < NUM_INPUTS; j++)
				inputs[in][i * NUM_INPUTS + j] = dst[DSP_SAMPLE_SIZE*in + j]/32768.0;
		}
	}
	double x_full[ADAPTATION_FILTER_N];
	for(int in = 0; in < ADAPTATION_FILTER_N; in++){
		x_full[in] =  inputs[in][REFERENCE_CHANNEL];

		//load data to model
		init_models();
/********************************************************FEEDBACK NEUTRALIZATION**********************************************************/
		for(int j=0; j < CONTROL_N; j++){
	
			for (int i = 0; i < 8; i++) {
				lf_ring_init(feedback_ref + i, 256);
				lf_ring_set_buffer(feedback_ref + i, NULL, 0);
			}

			//TODO replace 0 with next probes
			for (int i = 0; i < plate_params.u; i++)
				lf_ring_add(&feedback_ref[i], outputs[0][j][i]);
		
			x_full[in] -= feedback_neutralization(j);
		}
	}
/*****************************************************FILTER REFERENCY WITH SECONDARY PATH MODEL*****************************************/

for (int num_channel = 0; num_channel < CONTROL_N; num_channel++){

	//alokacja miejsca na model s path
	fxlms_initialize_s();

	//set s from models
	for(int ei = 0; ei < plate_params.e; ei++){
		for(int ui = 0; ui < plate_params.u; ui++){
			s[num_channel][ei][ui] = models[num_channel][ui][num_errors[ei]];
		}
	}

	//init x to lfir
	lf_ring_init(&x, REFERENCE_N);
	lf_ring_set_buffer(&x, NULL, 0);
	//TODO replace 0 with next probes
	lf_ring_add(&x, x_full[0]);

	fxlms_initialize_r();
	///////INIT AND LOAD E
	e = malloc(plate_params.e * sizeof(*e));
	for (int ei = 0; ei < plate_params.e; ei++) {
		lf_ring_init(&e[ei], SEC_FILTER_N);
		lf_ring_set_buffer(&e[ei], NULL, 0);
	}

	//TODO replace 0 with next probes
	for(int ei = 0; ei < plate_params.e; ei++ ){
		printf("error: %lf\n ", inputs[0][num_errors[ei]]);
		lf_ring_add(&e[ei], inputs[0][num_errors[ei]]);
	}
	/////////////
//filter reference signal with s
	int xi = 0;
	do {
		int ei = 0;
		do {
			int ui = 0;
			do {
				double r_temp;

				r_temp = lf_fir(&x, s[num_channel][ei][ui], SEC_FILTER_N);
				lf_ring_add(&r[num_channel][xi][ei][ui], r_temp);
				printf("%1.15lf\n",r_temp);
				ui++;
			} while (ui < plate_params.u);
		} while (++ei < plate_params.e);
		xi++;
	} while (xi < plate_params.x);


}

double mu = fxlms_normalize(r);
printf("norm: %lf\n",mu );
/**************************************************************************************************************************************/


/*******************************************WRITE DATA - SHARED MEMORY ********************************************************/

// int8_t *dst_toio;
// int8_t *dst;
// dst_toio = malloc(256);
// dst = malloc(256);
// memset(dst_toio, 3, 256);
// send_to_dsp(dst_toio);

// memcpy_fromio(dst, shm_save, DSP_WEIGHT_PACKAGE_SIZE);
// 	for(int i =0; i< 12; i++)
// 		printf("%d\n", dst[i]);

/**************************************************************************************************************************************/

return 0;
}
 
