#define CONTROL_N		        5	
#define SEC_FILTER_N		    128
#define ADAPTATION_FILTER_N 	    256
#define CONTROL_CHANNELS        4
#define ERROR_CHANNELS          9
static int num_errors[] = {1,2,3,9, 10, 11,19, 27, 35};
//static int num_errors[] = {1,2,3,9, 10, 11};

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
	.adaptation_n	= ADAPTATION_FILTER_N,
	.secondary_n	= 128,
	.feedback_n	= 128,
	.mu		= 0.0005,
//	.mu	= 0.01,
//	.mu = 1/
	.zeta	= 1e-6,
};

static double fxlms_normalize(struct lf_ring ***const *r){

	unsigned int ui, xi, ei, ni, i;
	double power = 0;

	for (xi = 0; xi < plate_params.x; xi++) {
		for (ei = 0; ei < plate_params.e; ei++) {
			for (ni = 0; ni < CONTROL_N; ni++) {
				for (ui = 0; ui < plate_params.u; ui++) {
					for (i = 0; i < ADAPTATION_FILTER_N * 2; i++){
						double t = lf_ring_get(&r[xi][ei][ni][ui], i);
						power += t * t;
					}
				}
			}
		}
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
		e_com[i] = 0.0;
		e_com[ADAPTATION_FILTER_N + i] = lf_ring_get(e, i); 
	}
}

complex calculate_alfa(complex *e_com, complex *r_com, complex *alfa){
	for(int i = 0; i < ADAPTATION_FILTER_N * 2; i++){
		alfa[i] = conj(r_com[i]) * e_com[i];
	}
}
