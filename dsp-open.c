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

#define DSP_WEIGHT_PACKAGE_SIZE 0X100
#define DSP_WEIGHT_OFFSET 0X500

unsigned long long shm_offset = 0x1000000;

static const char *dsp_device = "/dev/ds1104-0-mem";
static const uint8_t *shm, *shm_save;

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

static int dsp_remap(unsigned long size)
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

static int dsp_begin()
{

	int ret;
	uint32_t x;

	x = readl(shm + DSP_SHM_SIZE);
	if(x < DSP_RING_BASE){
		fprintf(stderr, "too small shared memory region: %d KiB\n", (unsigned int)x);
		return -EINVAL;
	}

	ret = dsp_remap(x);
	if(ret)
		return ret;

	return 0;
}

void copy_from_dsp(void * dst)
{
	unsigned int dsp_seq;
	int ret;

	ret = dsp_begin();
	dsp_seq = dsp_seqnum();
	__copy_from_dsp(dsp_seq, dst );
	show_data(dst, dsp_seq);
}

void send_to_dsp(int8_t *dst)
{
	
	void *filty;
	int ret;
	int fd; 

	fd = open(dsp_device, O_RDWR);
	if(fd == -1){
		perror("cannot open DSP device");
	//	ret = -errno;
	//	goto out;
	}

	filty = mmap(NULL, 0xf0000, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0xf00000);

	if(filty==MAP_FAILED){
		perror("cannot map DSP shared memory");
	//ret = -errno;
	//goto out_close;
	
	}

	shm_save = filty;

	memcpy_toio(filty, dst, DSP_WEIGHT_PACKAGE_SIZE);
}

static unsigned int error_mic_to_input(unsigned int p)
{
	return (p << 3) + 3;
}

const float ***s;
struct lf_ring ***r;
static struct lf_ring x;
static struct lf_ring feedback_ref[8];
static const int un = 4;
static const int xn = 1;
static const int en = 1; //jeden ?

#define REFERENCE_N		512
#define SEC_FILTER_N 128
#define FEEDBACK_N	64
#define CONTROL_N	5
#define REFERENCE_CHANNEL 0

static double feedback_neutralization()
{
	unsigned int ui;
	double f = 0;

	for (ui = 0; ui < un; ui++)
		f = lf_fir(&feedback_ref[ui], models[ui][REFERENCE_CHANNEL], FEEDBACK_N);
	return f;
}

static void fxlms_initialize_r()
{
	r = malloc(xn * sizeof(*r));
	for (int xi = 0; xi < xn; xi++) {
		r[xi] = malloc(en * sizeof(**r));
		for (int ei = 0; ei < en; ei++) {
			r[xi][ei] = malloc(un * sizeof(***r));
			for (int ui = 0; ui < un; ui++) {
				lf_ring_init(&r[xi][ei][ui], SEC_FILTER_N);
				lf_ring_set_buffer(&r[xi][ei][ui], NULL, 0);
			}
		}
	}
}	

int main() {

	//load_data_from_memory from all cards
	//outputs[5][8]
	//inputs[40]
	//alloc memory

	// if(dsp_remap(DSP_RING_BASE))
	//  printf(stderr, "cannot initialize DSP communication");

//	int16_t *dst = malloc(4096);
//		copy_from_dsp(dst);	

	//TODO set input[0] value on init
	double x_full = 20.0;

	for(int j=0; j < CONTROL_N; j++){

		//load data to models
		node_id = j;
		init_models();

		for (int i = 0; i < 8; i++) {
			lf_ring_init(feedback_ref + i, 256);
			lf_ring_set_buffer(feedback_ref + i, NULL, 0);
		}

		//TODO add outputs from dsp shared memory instead of 1.0
		for (int i = 0; i < un; i++)
			lf_ring_add(&feedback_ref[i], 1.0);
		
		x_full -= feedback_neutralization();
		printf("%1.15lf\n", x_full);
	}

/*****************************************************FILTER REFERENCY WITH SECONDARY PATH MODEL*****************************************/

for (int j=0; j < 1; j++){
//for (int j=0; j < CONTROL_N; j++){}

	node_id = j;
	init_models();
	//alokacja miejsca na model s path
	s = malloc(en * sizeof(*s));
	for (int ei = 0; ei < en; ei++)
		s[ei] = malloc(un * sizeof(**s));

	//set s from models
	for(int ui=0; ui<un; ui++){
		s[0][ui] = models[ui][error_mic_to_input(node_id)];
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
				printf("%lf\n",r_temp);
				ui++;
			} while (ui < un);
		} while (++ei < en);
		xi++;
	} while (xi < xn);
}
/**************************************************************************************************************************************/


/*******************************************READ AND WRITE DATA - SHARED MEMORY ********************************************************/

////////////////////READ/////////////////////////////////
// 	if(dsp_remap(DSP_RING_BASE))
// 		printf(stderr, "cannot initialize DSP communication");

// //	int16_t *dst = malloc(4096);

// //	for(int k=0;k<100;k++)
// //		copy_from_dsp(dst);	

////////////////////WRITE//////////////////////////////

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
