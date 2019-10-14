#define DSP_ALIGMENT                0x1000
#define DSP_RING_BASE               0x1000
//#define DSP_PACKAGE_SIZE          0x1000
#define DSP_PACKAGE_SIZE	    4096
#define DSP_SEQNUM                  0x08
#define DSP_SHM_SIZE		    0x20
//#define DSP_WEIGHT_PACKAGE_SIZE     128*4*sizeof(float)
#define DSP_RING_ENTRIES            0x18
#define DSP_CONTROL_OFFSET          0x04
#define DSP_WEIGHT_DATA_OFFSET      0x20
#define DSP_WRITE_DATA_OFFSET       0xf00000  

unsigned long long shm_offset = 0x1000000;

const void *shm_ringbuf[5];
void *shm_control[5];

static  unsigned int get_package_size(int n){
	
	return n * DSP_PACKAGE_SIZE;
}

static unsigned int get_weight_package_size(int n, int u){
	return n * u * sizeof(float); 
}

static unsigned int dsp_seqnum(int node_id)
{
	uint32_t x;

	x = readl(shm_ringbuf[node_id] + DSP_SEQNUM);
	return x;
}

static unsigned int dsp_ring_entries(int node_id)
{
	uint32_t x;
	x = readl(shm_ringbuf[node_id] + DSP_RING_ENTRIES);
	return x;
}

const uint8_t *dsp_get_buffer(unsigned int seq, unsigned int node_id)
{
	unsigned long offset;
	seq %= dsp_ring_entries(node_id);
	offset = seq * DSP_ALIGMENT;

	return shm_ringbuf[node_id] + DSP_RING_BASE + offset;
}

void __copy_from_dsp(unsigned int seq, void *dst, int n, int node_id)
{
#if 0
	const uint8_t *buffer = dsp_get_buffer(seq, node_id);

	memcpy_fromio(dst, buffer, get_package_size(n));
#else
	unsigned int entries = dsp_ring_entries(node_id);
	const uint8_t *buffer;
	size_t chunk;

	seq %= entries;
	chunk = n;
	if (chunk > entries - seq)
		chunk = entries - seq;

       	buffer = shm_ringbuf[node_id] + DSP_RING_BASE + seq * DSP_ALIGMENT;
	memcpy_fromio(dst, buffer, chunk * DSP_ALIGMENT);
	n -= chunk;
	if (n > 0) {
		buffer = shm_ringbuf[node_id] + DSP_RING_BASE;
		memcpy_fromio((char *) dst + chunk * DSP_ALIGMENT, buffer,
				n * DSP_ALIGMENT);
	}
#endif
}

static int dsp_remap(unsigned long size, unsigned int node_id)
{
	char dsp_dev[40];
	void *p;
	int ret;
	int fd;

	snprintf(dsp_dev, sizeof(dsp_dev), "/dev/ds1104-%d-mem", node_id);

	fd = open(dsp_dev, O_RDONLY);
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

	shm_ringbuf[node_id] = p;
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
//		printf("%d: ", seq);
			for(int j=0; j<16 ; j++)
			{	
//				double x = dst[16*i+j]/32768.0;
				double x = dst[j]/32768.0;
				printf("%f ",x);
			}
//		printf("\n");
//
//	}
}

static int dsp_begin(int node_id)
{

	int ret;
	uint32_t x;

	x = readl(shm_ringbuf[node_id] + DSP_SHM_SIZE);
	if(x < DSP_RING_BASE){
		fprintf(stderr, "too small shared memory region: %d KiB\n", (unsigned int)x);
		return -EINVAL;
	}

	ret = dsp_remap(x, node_id);
	if(ret)
		return ret;

	return 0;
}

void copy_from_dsp(void * dst, int node_id, unsigned int dsp_seq, int n)
{
	int ret;

	ret = dsp_begin(node_id);
	__copy_from_dsp(dsp_seq, dst, n, node_id);
//	show_data(dst, dsp_seq);
}

void send_to_dsp(void *dst, unsigned int weight_n, unsigned int control_n, int node_id)
{
	
	void *filty;
	int ret;
	int fd; 
	char dsp_dev[40];
	snprintf(dsp_dev, sizeof(dsp_dev), "/dev/ds1104-%d-mem", node_id);

	fd = open(dsp_dev, O_RDWR);
	if(fd == -1){
		perror("cannot open DSP device");
	}

	filty = mmap(NULL, 0xf0000, PROT_READ|PROT_WRITE, MAP_SHARED, fd, DSP_WRITE_DATA_OFFSET);

	if(filty==MAP_FAILED){
		perror("cannot map DSP shared memory");
	}

	shm_control[node_id] = filty;

	writel(weight_n, filty);
	writel(control_n, filty + DSP_CONTROL_OFFSET);	
	memcpy_toio(filty + DSP_WEIGHT_DATA_OFFSET, dst, get_weight_package_size(weight_n, control_n));
	munmap(filty, 0xf0000);
	close(fd);
}

int dsp_init(unsigned int node_id)
{
	if(dsp_remap(DSP_RING_BASE, node_id))
		fprintf(stderr, "cannot initialize DSP communication");
}

