

#ifndef INC_IIR_LOW_PASS_FILTER_H_
#define INC_IIR_LOW_PASS_FILTER_H_



#endif /* INC_IIR_LOW_PASS_FILTER_H_ */
typedef struct{
	float alpha;
	float beta;
	float prev_output; //y[n-1]
	float prev_input;  //x[n-1]
} IIR_Filter_1D;

typedef struct{
	IIR_Filter_1D x;
	IIR_Filter_1D y;
	IIR_Filter_1D z;
} IIR_Filter_3D;

void IIR_Filter_1D_Init(IIR_Filter_1D *f, float alpha, float beta);
float IIR_Filter_1D_Update(IIR_Filter_1D *f, float input);
void IIR_Filter_3D_Init(IIR_Filter_3D *f, float alpha, float beta);
void IIR_Filter_3D_Update(IIR_Filter_3D *f, float x_in, float y_in, float z_in, float *x_out, float *y_out, float *z_out);
