#include "osqp.h"
#define INF 2147483640
int main(int argc, char **argv) {
    // Load problem data
	float x=0.3;
	float x_dot=0.5;
	
	float k1=2;
	float k2=1;
	
	float upper_bound[2] = {k1-k1*x-k2*x_dot,k1+k1*x+k2*x_dot};
	float des_x = 2.0;
	float des_y = 0;
	float des_z = 0;
	float q_d[3] = {-des_x,-des_y,-des_z};

	printf("ub1: %f\n",upper_bound[0]);
	printf("ub2:%f\n",upper_bound[1]);

    c_float P_x[3] = {1.0,1.0,1.0,  };
    c_int P_nnz = 3;
    c_int P_i[3] = {0,1,2, };
    c_int P_p[4] = {0,1,2,3, };
    c_float q[3] = {q_d[0],q_d[1],q_d[2], };	//desired acc
    c_float A_x[2] = {1,-1, };
    c_int A_nnz = 2;
    c_int A_i[2] = {0,1, };
    c_int A_p[4] = {0,2,2,2, };
    c_float l[2] = {-100,-INF, };
    c_float u[2] = {upper_bound[0],upper_bound[1],};
    c_int n = 3;
    c_int m = 2;
    // Exitflag
    c_int exitflag = 0;

    // Workspace structures
    OSQPWorkspace *work;
    OSQPSettings  *settings = (OSQPSettings *)c_malloc(sizeof(OSQPSettings));
    OSQPData      *data     = (OSQPData *)c_malloc(sizeof(OSQPData));

    // Populate data
    if (data) {
        data->n = n;
        data->m = m;
        data->P = csc_matrix(data->n, data->n, P_nnz, P_x, P_i, P_p);
        data->q = q;
        data->A = csc_matrix(data->m, data->n, A_nnz, A_x, A_i, A_p);
        data->l = l;
        data->u = u;
    }

    // Define solver settings as default
    if (settings) {
        osqp_set_default_settings(settings);
        settings->alpha = 1.0; // Change alpha parameter
    }

    // Setup workspace
    exitflag = osqp_setup(&work, data, settings);

	printf("hello");
    
// Solve Problem
    osqp_solve(work);
	
	printf("%f \t %f \t %f \n",work->solution->x[0],work->solution->x[1],work->solution->x[2]);
    
	q[0] = -5;
	q[1] = 0;
	q[2] = 0;	//desired acc
	osqp_update_lin_cost(work,q);
	osqp_update_bounds(work,l,u);

    osqp_solve(work);
	printf("%f \t %f \t %f \n",work->solution->x[0],work->solution->x[1],work->solution->x[2]);
   
 // Cleanup
    if (data) {
        if (data->A) c_free(data->A);
        if (data->P) c_free(data->P);
        c_free(data);
    }
    if (settings) c_free(settings);

    return exitflag;

};
