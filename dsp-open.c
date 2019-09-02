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
#define DSP_SAMPLE_SIZE 0x14
#define DSP_SEQNUM 0x08
#define DSP_RING_BASE 0x1000
#define DSP_PACKAGE_SIZE 0x1000
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

	//for(int i=0; i<128;i++)
	//{
		printf("%d: ", seq);
			for(int j=0; j<16 ; j++)
			{	
				double x = dst[16+j]/32768.0;
				printf("%f ",x);
			}
		printf("\n");
	//}
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

void copy_from_dsp(void * dst, char *dsp_device)
{
	unsigned int dsp_seq;
	int ret;

	ret = dsp_begin(dsp_device);
	dsp_seq = dsp_seqnum();
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
#define SEC_FILTER_N 128
#define FEEDBACK_N	64
#define CONTROL_N	5
#define REFERENCE_CHANNEL 0

const float ***s;
struct lf_ring ***r;
static struct lf_ring x;
static struct lf_ring feedback_ref[8];
//static const int un = 4;
//static const int xn = 1;
//static const int en = 1; //jeden ?

static struct fxlms_params plate_params = {
	.u	= 4,
	.e	= 9,
	.x	= 1,
	.m	= 128,
	.n	= 128,
	.mu	= 0.001,
	.zeta	= 1e-6,
};

static double feedback_neutralization()
{
	unsigned int ui;
	double f = 0;

	for (ui = 0; ui < plate_params.u ; ui++)
		f = lf_fir(&feedback_ref[ui], models[ui][REFERENCE_CHANNEL], FEEDBACK_N);
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

static double fxlms_normalize(const struct lf_ring ***r){
	
	unsigned int ui = 0;
	unsigned int xi = 0;	
	unsigned int i = 0;
	double power = 0;

	//TODO what should be in ei?
	unsigned int ei = 0;
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
		}while(++ui<plate_params.u);
	}while(++xi<plate_params.x);
	return plate_params.mu / (power + plate_params.zeta);
};	


int main() {

	//load_data_from_memory from all cards
	float outputs[CONTROL_N][NUM_OUTPUTS];
	float inputs[NUM_ALL_INPUTS];
	
	int16_t *dst;

	for(int i=0; i < CONTROL_N; i++)
	{
		snprintf(dsp_device, sizeof(dsp_device), "/dev/ds1104-%d-mem", i);
		if(dsp_remap(DSP_RING_BASE,dsp_device));
 			printf(stderr, "cannot initialize DSP communication");
	
		dst = malloc(4096);
		copy_from_dsp(dst,dsp_device);
		for(int j = 0; j < NUM_OUTPUTS; j++)
		     outputs[i][j] = dst[NUM_INPUTS+j]/32768.0;	
		for(int j = 0; j < NUM_INPUTS; j++)
			inputs[i * NUM_INPUTS + j] = dst[j]/32768.0;

	}
	double x_full = inputs[REFERENCE_CHANNEL];


/********************************************************FEEDBACK NEUTRALIZATION**********************************************************/
	for(int j=0; j < CONTROL_N; j++){

		//load data to models
		node_id = j;
		init_models();

		for (int i = 0; i < 8; i++) {
			lf_ring_init(feedback_ref + i, 256);
			lf_ring_set_buffer(feedback_ref + i, NULL, 0);
		}

		//TODO distinguish outputs from different control channels
		for (int i = 0; i < plate_params.u; i++)
			lf_ring_add(&feedback_ref[i], *outputs[i]);
		
		x_full -= feedback_neutralization();
	}

/*****************************************************FILTER REFERENCY WITH SECONDARY PATH MODEL*****************************************/

for (int j=0; j < 1; j++){
//for (int j=0; j < CONTROL_N; j++){}

	node_id = j;
	init_models();
	//alokacja miejsca na model s path
	s = malloc(plate_params.e * sizeof(*s));
	for (int ei = 0; ei < plate_params.e; ei++)
		s[ei] = malloc(plate_params.u * sizeof(**s));

	//set s from models
	for(int ei = 0; ei < plate_params.e; ei++){
		for(int ui=0; ui<plate_params.u; ui++){
			s[ei][ui] = models[ui][error_mic_to_input(node_id)];
		}
	}
	lf_ring_init(&x, REFERENCE_N);
	lf_ring_set_buffer(&x, NULL, 0);
	lf_ring_add(&x, x_full);

	fxlms_initialize_r();

	//filter reference signal with s
	int xi = 0;
	do {
		int ei = 0;
		do {
			int ui = 0;
			do {
				double r_temp;

				r_temp = lf_fir(&x, s[ei][ui], SEC_FILTER_N);
				lf_ring_add(&r[xi][ei][ui], r_temp);
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
 
