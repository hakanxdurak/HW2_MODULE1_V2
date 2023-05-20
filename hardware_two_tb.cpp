#include <hls_stream.h>
#include <ap_int.h>
#include <stdio.h>
#include <cmath>
#define GPR_SIZE 46848
#define W_SIZE 256
#define H_SIZE 183
using namespace hls;

struct axis_data{
	float data;
	ap_uint<1> last;
};

void module_1_hw(stream<axis_data> &TARGET, stream<axis_data> &X, stream<axis_data> &W_IN, stream<axis_data> &H, stream<axis_data> &W_OUT);
void software(float TARGET[GPR_SIZE], float X[GPR_SIZE], float W_IN[W_SIZE], float H[H_SIZE], float W_OUT[GPR_SIZE]);

int main(){
	int fail_flag = 0;
	int err = 0;

	float TARGET[GPR_SIZE];
	float X[GPR_SIZE];
	float W_IN[W_SIZE];
	float H[H_SIZE];

	for(int i = 0 ; i < GPR_SIZE ; i++){
		TARGET[i] = i * 0.0001235;
		X[i] = i * 0.02342335;
	}
	for(int i = 0 ; i < W_SIZE ; i++){
		W_IN[i] = i * 0.05637435;
	}
	for(int i = 0 ; i < H_SIZE ; i++){
		H[i] = i * 0.346521235;
	}
/*                  TARGET                  */
	axis_data local_read_target;
	hls::stream<axis_data> target_hw;
	//from right to left not from up to bottom
	float TARGET_temp[GPR_SIZE];
	int target_tmp;
	int k = 0;
	for(int i = 0 ; i < W_SIZE ; i++){
		for(int j = 0 ; j < H_SIZE ; j++){
			target_tmp = i + (j << 8);
			TARGET_temp[k] = TARGET[target_tmp];
			k++;
		}
	}
	for(int i = 0 ; i < GPR_SIZE ; i++){
		local_read_target.data = TARGET_temp[i];
		if(i == (GPR_SIZE-1))
			local_read_target.last = (ap_uint<1>)1;
		else
			local_read_target.last = (ap_uint<1>)0;
		target_hw.write(local_read_target);
	}
/*                  X                  */
	axis_data local_read_x;
	hls::stream<axis_data> x_hw;
	//from right to left not from up to bottom(same as TARGET)
	float X_temp[GPR_SIZE];
	k = 0;
	for(int i = 0 ; i < W_SIZE ; i++){
		for(int j = 0 ; j < H_SIZE ; j++){
			target_tmp = i + (j << 8);
			X_temp[k] = X[target_tmp];
			k++;
		}
	}
	for(int i = 0 ; i < GPR_SIZE ; i++){
		local_read_x.data = X_temp[i];
		if(i == (GPR_SIZE-1))
			local_read_x.last = (ap_uint<1>)1;
		else
			local_read_x.last = (ap_uint<1>)0;
		x_hw.write(local_read_x);
	}
/*                  H                  */
	axis_data local_read_h;
	hls::stream<axis_data> h_hw;
	//recursive from 0 to 183, for all 256 lines
	float H_temp[GPR_SIZE];
	k = 0;
	for(int i = 0 ; i < W_SIZE ; i++){
		for(int j = 0 ; j < H_SIZE ; j++){
			H_temp[k] = H[j];
			k++;
		}
	}
	for(int i = 0 ; i < GPR_SIZE ; i++){
		local_read_h.data = H_temp[i];
		if(i == (GPR_SIZE-1))
			local_read_h.last = (ap_uint<1>)1;
		else
			local_read_h.last = (ap_uint<1>)0;
		h_hw.write(local_read_h);
	}
/*                  W                  */
	axis_data local_read_w;
	hls::stream<axis_data> w_in_hw;
	//same value for 0(183 times),same value for 1(183 times),...
	float W_IN_temp[GPR_SIZE];
	k = 0;
	for(int i = 0 ; i < W_SIZE ; i++){
		for(int j = 0 ; j < H_SIZE ; j++){
			W_IN_temp[k] = W_IN[i];
			k++;
		}
	}
	for(int i = 0 ; i < GPR_SIZE ; i++){
		local_read_w.data = W_IN_temp[i];
		if(i == (GPR_SIZE-1))
			local_read_w.last = (ap_uint<1>)1;
		else
			local_read_w.last = (ap_uint<1>)0;
		w_in_hw.write(local_read_w);
	}
/*                  end                  */

	hls::stream<axis_data> w_out_hw;
	module_1_hw(target_hw, x_hw, w_in_hw, h_hw, w_out_hw);
	float W_OUT_sw[W_SIZE];
	software(TARGET, X, W_IN, H, W_OUT_sw);

/*                  W_OUT_HW                  */
	axis_data local_read_out;
	float W_OUT_hw[W_SIZE];
	for(int i = 0 ; i < W_SIZE ; i++){
		for(int j = 0 ; j < H_SIZE ; j++){
			local_read_out = w_out_hw.read();
			if(j == (H_SIZE - 1)){
				W_OUT_hw[i] = local_read_out.data;
			}
		}
	}
/*                  end                  */

	for(int i = 0 ; i < W_SIZE ; i++){
		if(W_OUT_hw[i] != W_OUT_sw[i]){
			printf("Failed at [%d] | Hw:[%f] Sw:[%f]\n", i, W_OUT_hw[i], W_OUT_sw[i]);
			err++;
			fail_flag = 1;
		}
	}
	printf("Total error: %d\n", err);
	if(fail_flag == 1)
		printf("FAILED\n");
	else
		printf("PASSED\n");
	return fail_flag;
}
void software(float TARGET[GPR_SIZE], float X[GPR_SIZE], float W_IN[W_SIZE], float H[H_SIZE], float W_OUT[GPR_SIZE]){
	float MSXHt[W_SIZE];
	float norms, b;
	int i0, i1, target_tmp;
	for(i0 = 0 ; i0 < W_SIZE ; i0++){
		norms = 0.0;
		b = 0.0;
		for(i1 = 0 ; i1 <H_SIZE ; i1++){
			target_tmp = i0 + (i1 << 8);
			norms = norms + TARGET[target_tmp] * H[i1];
			b = b + X[target_tmp] * H[i1];
		}
		norms = norms - b;
		MSXHt[i0] = -norms;
		if(norms > 0.0)
			MSXHt[i0] = 0.0;
	}
	norms = 0.0;
	for(i1 = 0 ; i1 < H_SIZE ; i1++){
		norms = norms + H[i1] * H[i1];
	}
	for(i0 = 0 ; i0 < W_SIZE ; i0++){
		W_OUT[i0] = MSXHt[i0] * W_IN[i0]/fmaxf(W_IN[i0] * norms, 1.0E-20);
	}
}


