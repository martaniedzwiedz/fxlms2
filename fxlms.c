#define CONTROL_N		        5	
#define SEC_FILTER_N		    128
#define ADAPTATION_FILTER_N 	128
#define CONTROL_CHANNELS        4
#define ERROR_CHANNELS          9
static int num_errors[9] = {1,2,3,9, 10, 11,19, 27, 35};

struct fxlms_params
{
	double mu;
	double zeta;
	unsigned int u;
	unsigned int e;
	unsigned int x;
	unsigned int adaptation_n; //adaptation filter
	unsigned int secondary_n; //secondary filter
	unsigned int feedback_n; //feedback filter
};
static struct fxlms_params plate_params = {
	.u	= CONTROL_CHANNELS,
	.e	= ERROR_CHANNELS,
	.x	= 1,
	.adaptation_n	= 128,
	.secondary_n	= 128,
	.feedback_n	= 64,
	.mu	= 0.01,
	.zeta	= 1e-6,
};

static double fxlms_normalize(const struct lf_ring ***r){

	unsigned int ui = 0;
	unsigned int xi = 0;	
	float power = 0;

	unsigned int ei = 0;

	for(int i = 0; i < ADAPTATION_FILTER_N * 2; i++){
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
