#define DSP_ALIGMENT                0x2000
#define DSP_RING_BASE               0x1000
#define DSP_PACKAGE_SIZE            0x2000
#define DSP_SEQNUM                  0x08
#define DSP_SHM_SIZE		        0x20
#define DSP_WEIGHT_PACKAGE_SIZE     128*4*2
#define DSP_RING_ENTRIES            0x18
#define DSP_CONTROL_OFFSET          0x04
#define DSP_WEIGHT_DATA_OFFSET      0x20
#define DSP_WRITE_DATA_OFFSET       0xf00000  

static const uint8_t *shm, *shm_save;
unsigned long long shm_offset = 0x1000000;

static unsigned int dsp_seqnum()
{
	uint32_t x;

	x = readl(shm + DSP_RING_ENTRIES);
	return x;
}

static unsigned int dsp_ring_entries()
{
	uint32_t x;
	x = readl(shm + DSP_SEQNUM);
	return x;
}

unsigned int *dsp_get_buffer(unsigned int seq)
{
	unsigned long offset;
	seq %= dsp_ring_entries();
	offset = seq * DSP_ALIGMENT;
	return shm + DSP_RING_BASE + offset;
}

void __copy_from_dsp(unsigned int seq, void *dst)
{
	const uint8_t *buffer = dsp_get_buffer(seq);
	memcpy_fromio(dst, buffer, DSP_PACKAGE_SIZE);
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

//	for(int i=0; i<256;i++)
//	{
		printf("%d: ", seq);
			for(int j=0; j<16 ; j++)
			{	
//				double x = dst[16*i+j]/32768.0;
				double x = dst[j]/32768.0;
				printf("%f ",x);
			}
		printf("\n");
//	}
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
	int ret;

	ret = dsp_begin(dsp_device);
	__copy_from_dsp(dsp_seq, dst );
	show_data(dst, dsp_seq);
}

void send_to_dsp(int8_t *dst, char *dsp_device, unsigned int weight_n, unsigned int control_n)
{
	
	void *filty;
	int ret;
	int fd; 

	fd = open(dsp_device, O_RDWR);
	if(fd == -1){
		perror("cannot open DSP device");
	}

	filty = mmap(NULL, 0xf0000, PROT_READ|PROT_WRITE, MAP_SHARED, fd, DSP_WRITE_DATA_OFFSET);

	if(filty==MAP_FAILED){
		perror("cannot map DSP shared memory");
	}

	shm_save = filty;

	writel(weight_n, filty);
	writel(control_n, filty + DSP_CONTROL_OFFSET);	
	memcpy_toio(filty + DSP_WEIGHT_DATA_OFFSET, dst, DSP_WEIGHT_PACKAGE_SIZE);
}
