#define CONTROL_N		        5	
#define SEC_FILTER_N		    128
#define ADAPTATION_FILTER_N 	128
static int num_errors[9] = {1,2,3,9, 10, 11,19, 27, 35};

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
static struct fxlms_params plate_params = {
	.u	= 4,
	.e	= 9,
	.x	= 1,
	.m	= 128,
	.n	= 128,
	.mu	= 0.001,
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

static void fxlms_initialize_r(struct lf_ring ***r)
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

static void fxlms_initialize_s(const float ****s)
{
	s = malloc(CONTROL_N * sizeof(*s));
	for(int num_channels = 0; num_channels < CONTROL_N; num_channels++){
	s[num_channels] = malloc(plate_params.e * sizeof(**s));
	for (int ei = 0; ei < plate_params.e; ei++)
		s[num_channels][ei] = malloc(plate_params.u * sizeof(***s));
	}
}	

static void fxlms_initialize_e(struct lf_ring *e){
	e = malloc(plate_params.e * sizeof(*e));
	for (int ei = 0; ei < plate_params.e; ei++) {
		lf_ring_init(&e[ei], SEC_FILTER_N);
		lf_ring_set_buffer(&e[ei], NULL, 0);
	}
}

static void fxlms_initialize_model(const float ****s){

	fxlms_initialize_s(s);

	for (int num_channel = 0; num_channel < CONTROL_N; num_channel++){
		for(int ei = 0; ei < plate_params.e; ei++){
			for(int ui = 0; ui < plate_params.u; ui++){
				s[num_channel][ei][ui] = models[num_channel][ui][num_errors[ei]];
			}
		}
	}    
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
