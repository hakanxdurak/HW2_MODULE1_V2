#include <hls_stream.h>
#include "ap_int.h"
#include <cmath>
#define GPR_SIZE 46848
#define W_SIZE 256
#define H_SIZE 183
using namespace hls;

struct axis_data{
	float data;
	ap_uint<1> last;
};

void hardware_two(stream<axis_data> &TARGET, stream<axis_data> &X, stream<axis_data> &W_IN, stream<axis_data> &H, stream<axis_data> &W_OUT){
#pragma HLS INTERFACE ap_ctrl_none port=return
#pragma HLS INTERFACE axis register both port=W_OUT
#pragma HLS INTERFACE axis register both port=H
#pragma HLS INTERFACE axis register both port=W_IN
#pragma HLS INTERFACE axis register both port=TARGET
#pragma HLS INTERFACE axis register both port=X
	float norms;
	float b;
	float norms2;
	axis_data local_read_target;
	axis_data local_read_x;
	axis_data local_read_w, local_write_w;
	axis_data local_read_h;
	loop_W: for(int j = 0 ; j < W_SIZE ; j++){
#pragma HLS PIPELINE II = 1
		loop_H: for(int i = 0 ; i < H_SIZE ; i++){
			local_read_target = TARGET.read();
			local_read_h = H.read();
			local_read_x = X.read();
			local_read_w = W_IN.read();
			if(i == 0){
				norms = 0.0;
				b = 0.0;
				norms2 = 0.0;
			}
			norms = norms + local_read_target.data * local_read_h.data;
			b = b + local_read_x.data * local_read_h.data;
			norms2 = norms2 + local_read_h.data * local_read_h.data;
			if(norms > b)
				local_write_w.data = 0.0;
			else
				local_write_w.data = (b - norms) * local_read_w.data/fmaxf(local_read_w.data * norms2, 1.0E-20);

			if((i == (H_SIZE-1)) && (j == (W_SIZE-1)))
				local_write_w.last = (ap_uint<1>)1;
			else
				local_write_w.last = (ap_uint<1>)0;
			W_OUT.write(local_write_w);
		}
	}
}

