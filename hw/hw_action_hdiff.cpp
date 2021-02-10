/*
 * Copyright 2019 International Business Machines
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/* SNAP HLS_HDIFF */

#include <string.h>
#include "ap_int.h"
#include "hw_action_hdiff.H"
#define HLINE "-------------------------------------------------------------\n"
#define TEST
#define TESTALLOC
#define TESTRESULT
//#define DEBUG


//**************************************************************************************************
//**************************************************************************************************
//**************************************************************************************************
//***************************************Horizontal Diffusion Stencil*******************************
//**************************************************************************************************
static void hdiff_2D_stream(mat_elmt_t data_in[COLUMNS][ROWS], mat_elmt_t data_out[COLUMNS][ROWS])
{
	#pragma HLS INLINE OFF

	float temp = 0;
	uint32_t x,y,z;


	actionloop_stencilColumn:for(int c=2;c<COLUMNS-2;c++){
			mat_elmt_t dataRow1[ROWS];
			mat_elmt_t dataRow2[ROWS];
			mat_elmt_t dataRow3[ROWS];
//			cache_stencilRow:for(int r=0;r<ROWS;r++)
//			{
//				dataRow1[r]=data_in[c][r];
//				dataRow2[r]=data_in[c-1][r];
//				dataRow3[r]=data_in[c+1][r];
//
//			}

		actionloop_stencilRow:for(int r=2;r<ROWS-2;r++){
			#pragma HLS PIPELINE

			//data_out[c][r]=data_in[c][r];
			
			double lap_ij = 4 *data_in[c][r] - data_in[c][r-1] - data_in[c][r+1] -
			data_in[c-1][r] - data_in[c+1][r];

			double lap_imj =  4 *data_in[c][r-1] - data_in[c][r-2] - data_in[c][r] -
			data_in[c-1][r-1] - data_in[c+1][r-1];
			double lap_ipj = 4 *data_in[c][r+1] - data_in[c][r] - data_in[c][r+2] -
			data_in[c-1][r+1] - data_in[c+1][r+1];
			double lap_ijm =  4 *data_in[c-1][r] - data_in[c-1][r-1] - data_in[c-1][r+1] -
			data_in[c-2][r] - data_in[c][r];

			double lap_ijp =  4 *data_in[c+1][r] - data_in[c+1][r-1] - data_in[c+1][r+1] -
			data_in[c][r] - data_in[c+2][r];


			double flx_ij = lap_ipj - lap_ij;
			flx_ij = flx_ij * (data_in[c][r+1] - data_in[c][r]) > 0 ? 0 : flx_ij;

			double flx_imj = lap_ij - lap_imj;
			flx_imj = flx_imj * (data_in[c][r] - data_in[c][r-1]) > 0 ? 0 : flx_imj;
			double fly_ij = lap_ijp - lap_ij;
			fly_ij = fly_ij * (data_in[c+1][r] - data_in[c][r]) > 0 ? 0 : fly_ij;

			double fly_ijm = lap_ij - lap_ijm;
			fly_ijm = fly_ijm * (data_in[c][r] - data_in[c-1][r]) > 0 ? 0 : fly_ijm;
			data_out[c][r] = data_in[c][r] - 7 * (flx_ij - flx_imj + fly_ij - fly_ijm);
			#ifdef TESTRESULT
				printf("FPAG stencil[%d,%d]::::%f\n",c,r,data_out[c][r]);
			#endif

			
		}
	}

}

void read_mem_to_stream (hls::stream<snap_membus_1024_t>& in_stream,
						snap_membus_1024_t *din_gmem,
						snapu64_t input_address,
						snapu64_t size_in_lines)
{
#pragma HLS INLINE OFF
	const int lines=MAX_NB_OF_LINES_READ;
	snap_membus_1024_t* temp_ptr = din_gmem + input_address;
	rd_din_loop: for (unsigned k=0; k < size_in_lines; k++) {
		//printf("size_in lines=%d\n",k);
	#pragma HLS PIPELINE
#pragma HLS LOOP_TRIPCOUNT max=lines
		in_stream.write(temp_ptr[k]);
	}
}



//assign complete one to acc1
// CACHELINE {0-99}--->>> ACC 0
// CACHELINE {100-199}--->>> ACC 1
void stream_splitter (
   hls::stream<snap_membus_1024_t>& bus_input,
   hls::stream<snap_membus_1024_t> output[MANY_ACC],
   unsigned num_input_lines)
  {
	const int lines=MAX_NB_OF_LINES_READ;
	const int calls_per_acc=TOTAL_MEMORY_LINES/MAX_NB_OF_LINES_READ/MANY_ACC;
	for (unsigned c = 0; c < calls_per_acc; c++)
	{
		for (unsigned p = 0; p < MANY_ACC; p++)
		{
			//printf("stream_splitter::ACCELERATOR:::%d\n",p);
			for (unsigned i = 0; i < lines; i ++)
			{
				//printf("stream_splitter:::::%d\n",i);
				#pragma HLS LOOP_TRIPCOUNT max=lines 
				#pragma HLS PIPELINE II=1
				snap_membus_1024_t temp = bus_input.read();
				output[p].write(temp);
			}
		}
	}
  }

  void stream_merger (
   hls::stream<snap_membus_1024_t>& bus_output,
   hls::stream<snap_membus_1024_t> output[MANY_ACC],
   unsigned num_output_lines)
  {
	snap_membus_1024_t temp;
	const int lines=MAX_NB_OF_LINES_READ;
	const int calls_per_acc=TOTAL_MEMORY_LINES/MAX_NB_OF_LINES_READ/MANY_ACC;
	for (unsigned c = 0; c < calls_per_acc; c++)
	{
		for (unsigned p = 0; p < MANY_ACC; p++) {
		 #pragma HLS UNROLL

			//printf("stream_splitter::ACCELERATOR:::%d\n",p);
			for (unsigned i = 0; i < lines; i ++)
			{
	#pragma HLS LOOP_TRIPCOUNT max=lines
			#pragma HLS PIPELINE II=1
			//	printf("stream_splitter:::::%d\n",i);

				temp = output[p].read();
				//  #pragma HLS LOOP_TRIPCOUNT max=lines avg=lines/2

				bus_output.write(temp);
			}
		}
	}
}

void stream_2D_grid(mat_elmt_t hdiff_in[MANY_ACC][COLUMNS][ROWS],
		 hls::stream<snap_membus_1024_t> dataTable[MANY_ACC],snapu64_t size_in_lines) {
	#pragma HLS INLINE OFF
	mat_elmt_t d=0;
	union {
		uint64_t     value_u;
		mat_elmt_t   value_d;
	};
	const int acc=MANY_ACC;
	const int lines=MAX_NB_OF_LINES_READ;
	const int calls_per_acc=TOTAL_MEMORY_LINES/MAX_NB_OF_LINES_READ/MANY_ACC;
	for (unsigned c = 0; c < calls_per_acc; c++)
	{
		for (unsigned p = 0; p < MANY_ACC; p++)
		{
			printf("calling acc::::%d\n",p);
	mat_elmt_t* temp_ptr = &hdiff_in[p][0][0];
 	 selection_loop: for (unsigned i = 0; i < size_in_lines; i++) {
 	 #pragma HLS PIPELINE II=1
  	//printf("steam_2D_grid:::cacheline=%d \n",i);
    	snap_membus_1024_t tempRead=dataTable[p].read();
    	HistogramFPGAStream_label2: for(int j = 0; j < 32; j++)
    	{
#pragma HLS PIPELINE II=1
     		value_u=tempRead.range((8*sizeof(mat_elmt_t)*(j+1))-1, (8*sizeof(mat_elmt_t)*j));
        		*(temp_ptr + ( i *32) + j)=value_d;



        		#ifdef TESTALLOC
			printf("GRIDDING:::[%d]::::%f\n",( i *32) + j,value_d);
		#endif


    	}
   
}
  	  }
}}


void grid_2D_stream(mat_elmt_t hdiff_out[MANY_ACC][COLUMNS][ROWS],
		hls::stream<snap_membus_1024_t> data_out[MANY_ACC], snapu64_t size_in_lines) {
	#pragma HLS INLINE OFF
	mat_elmt_t d=0;

	
	union {
	uint64_t     value_u;
	mat_elmt_t   value_d;
	};
	snap_membus_1024_t temp=0;
	const int acc=MANY_ACC;
	const int lines=MAX_NB_OF_LINES_READ;
	const int calls_per_acc=TOTAL_MEMORY_LINES/MAX_NB_OF_LINES_READ/MANY_ACC;
	for (unsigned c = 0; c < calls_per_acc; c++)
	{
		for (unsigned p = 0; p < MANY_ACC; p++)
		{
		#pragma HLS UNROLL
			printf("calling acc::::%d\n",p);
			mat_elmt_t* temp_ptr = &hdiff_out[p][0][0];
			selection_loop: for (unsigned i = 0; i < size_in_lines; i++)
			{
				#pragma HLS PIPELINE II=1
				HistogramFPGAStream_label2: for(int j = 0; j < 32; j++)
				{
				#pragma HLS UNROLL factor=32

					value_d=(mat_elmt_t)*(temp_ptr + ( i *32) + j);

						temp.range((32*(j+1))-1, (32*j))=value_u;
						
					

					#ifdef TESTALLOC

						printf("STREAMING:::[%d]::::%f\n",( i *32) + j,value_d);
					#endif
					//printf("SENDING DATA d[%d]=%f\n",i*32+j,*(temp_ptr + ( i *32) + j));

				}
				data_out[p].write(temp);
			}


		}
	}/// end acc loop
}


// WRITE DATA TO MEMORY
void write_stream_to_mem (hls::stream<snap_membus_1024_t>& out_stream,
						snap_membus_1024_t *dout_gmem,
						snapu64_t output_address,
						snapu64_t size_in_lines)
{
	const int lines=MAX_NB_OF_LINES_READ;
	snap_membus_1024_t* temp_ptr = dout_gmem + output_address;
	wb_dout_loop: for (unsigned k=0; k < size_in_lines; k++) {
#pragma HLS PIPELINE
#pragma HLS LOOP_TRIPCOUNT max=lines
		temp_ptr[k] = out_stream.read();
	}
}

void hdiff_in (snap_membus_1024_t *din_gmem, 
		mat_elmt_t data_in[MANY_ACC][COLUMNS][ROWS],
		snapu64_t input_address,
		snapu64_t size_in_lines)
{
	#pragma HLS INLINE OFF
	#pragma HLS DATAFLOW
	const int acc=MANY_ACC;
	hls::stream<snap_membus_1024_t> bus_in_stream("bus_in_stream");
	hls::stream<snap_membus_1024_t> in_stream[acc];
	
	read_mem_to_stream(bus_in_stream, din_gmem, input_address, size_in_lines);
	stream_splitter (bus_in_stream,in_stream, MAX_NB_OF_LINES_READ);
	stream_2D_grid(data_in,in_stream,MAX_NB_OF_LINES_READ);
	//fpga_execution(in_stream,data_out);


}

void hdiff_out (snap_membus_1024_t *dout_gmem, 
		mat_elmt_t data_out[MANY_ACC][COLUMNS][ROWS],
		snapu64_t output_address,
		snapu64_t size_in_lines)
{
	#pragma HLS INLINE OFF
	#pragma HLS DATAFLOW
	const int acc=MANY_ACC;
	hls::stream<snap_membus_1024_t> bus_out_stream("bus_out_stream");

	hls::stream<snap_membus_1024_t> out_stream[acc];
	
	grid_2D_stream(data_out,out_stream,MAX_NB_OF_LINES_READ) ;
	
	stream_merger (bus_out_stream,out_stream,MAX_NB_OF_LINES_READ);
	write_stream_to_mem(bus_out_stream, dout_gmem, output_address, size_in_lines);


}

//----------------------------------------------------------------------
//--- MAIN PROGRAM -----------------------------------------------------
//----------------------------------------------------------------------
static void process_action(snap_membus_1024_t *din_gmem,
                           snap_membus_1024_t *dout_gmem,
                           snap_membus_512_t *d_ddrmem,
                           action_reg *act_reg)
{
	// VARIABLES
	snapu32_t xfer_size;
	snapu32_t action_xfer_size;
	snapu32_t nb_blocks_to_xfer;
	snapu16_t i;
	short rc = 0;
	snapu32_t ReturnCode = SNAP_RETC_SUCCESS;
	snapu64_t InputAddress_1024;
	snapu64_t OutputAddress_1024;
	snapu64_t address_xfer_offset_1024;
	snapu64_t InputAddress_512;
	snapu64_t OutputAddress_512;
	snapu64_t address_xfer_offset_512;
	// if 4096 bytes max => 64 words

	// byte address received need to be aligned with port width
	InputAddress_1024 = (act_reg->Data.in.addr)   >> ADDR_RIGHT_SHIFT_1024;
	OutputAddress_1024 = (act_reg->Data.out.addr) >> ADDR_RIGHT_SHIFT_1024;
	InputAddress_512 = (act_reg->Data.in.addr)   >> ADDR_RIGHT_SHIFT_512;
	OutputAddress_512 = (act_reg->Data.out.addr) >> ADDR_RIGHT_SHIFT_512;

	address_xfer_offset_1024 = 0x0;
	// testing sizes to prevent from writing out of bounds
	action_xfer_size = MIN(act_reg->Data.in.size,
			       act_reg->Data.out.size);
	bool fpgaMem=false;
	unsigned size_in_lines;
	if (act_reg->Data.in.type == SNAP_ADDRTYPE_CARD_DRAM and
	    act_reg->Data.in.size > CARD_DRAM_SIZE) {
	        act_reg->Control.Retc = SNAP_RETC_FAILURE;
		return;
        }
	if (act_reg->Data.out.type == SNAP_ADDRTYPE_CARD_DRAM and
	    act_reg->Data.out.size > CARD_DRAM_SIZE) {
	        act_reg->Control.Retc = SNAP_RETC_FAILURE;
		return;
        }

	if (action_xfer_size%BPERDW_1024 == 0)
		size_in_lines = action_xfer_size/BPERDW_1024;
	else
		size_in_lines = (action_xfer_size/BPERDW_1024) + 1;

	if (act_reg->Data.in.type == SNAP_ADDRTYPE_CARD_DRAM and act_reg->Data.in.size > CARD_DRAM_SIZE) {
		act_reg->Control.Retc = SNAP_RETC_FAILURE;
		return;
	}

	printf("Inside action\n");
	uint32_t size;
	int kernelCall=TOTAL_MEMORY_LINES/MAX_NB_OF_LINES_READ;
	printf("kernelCall %d\n",kernelCall);

	const int acc=MANY_ACC;
	mat_elmt_t data_in[MANY_ACC][COLUMNS][ROWS];
	#pragma HLS ARRAY_PARTITION variable=data_in complete dim=1
	#pragma HLS ARRAY_PARTITION variable=data_in cyclic factor=32 dim=3

	mat_elmt_t data_out[MANY_ACC][COLUMNS][ROWS];
	#pragma HLS ARRAY_PARTITION variable=data_out complete dim=1
	#pragma HLS ARRAY_PARTITION variable=data_out cyclic factor=32 dim=3




	 hdiff_in (din_gmem, 
		data_in,InputAddress_1024+address_xfer_offset_1024,
		size_in_lines);

	printf("size_in_lines==%d\n",size_in_lines );
	printf("TOTAL_MEMORY_LINES==%d\n",TOTAL_MEMORY_LINES );

//	int calls_per_acc=TOTAL_MEMORY_LINES/size_in_lines/acc;
	int calls_per_acc=TOTAL_MEMORY_LINES/MAX_NB_OF_LINES_READ/acc;
	printf("calls_per_acc=%d\n",calls_per_acc);
	call_per_acc_loop:for (unsigned c = 0; c < calls_per_acc; c++)
	{
		acc_loop:for(unsigned ac=0;ac<acc;ac++)
		{
			#pragma HLS UNROLL complete
			printf("computation\n");

			hdiff_2D_stream(data_in[ac],data_out[ac]);
		// calls_per_acc--;
		}
	}

	// for(int a=0;a<acc;a++)
	// {
	// 	for(int c=0;c<COLUMNS;c++)
	// 	{

	// 		for(int r=0;r<ROWS;r++)
	// 		{
	// 		#pragma HLS PIPELINE II=1
	// 		printf("after computation::::%f\n",data_out[a][c][r]);

	// 		}
	// 	}
	// }
	 hdiff_out (dout_gmem, data_out,OutputAddress_1024+address_xfer_offset_1024,
		 size_in_lines);

}

//--- TOP LEVEL MODULE -------------------------------------------------
void hls_action(snap_membus_1024_t *din_gmem,
		snap_membus_1024_t *dout_gmem,
		snap_membus_512_t *d_ddrmem,
		action_reg *act_reg)
{
	// Host Memory AXI Interface
#pragma HLS INTERFACE m_axi port=din_gmem bundle=host_mem offset=slave depth=512  \
  max_read_burst_length=64  max_write_burst_length=64 
#pragma HLS INTERFACE s_axilite port=din_gmem bundle=ctrl_reg offset=0x030

#pragma HLS INTERFACE m_axi port=dout_gmem bundle=host_mem offset=slave depth=512 \
  max_read_burst_length=64  max_write_burst_length=64 
#pragma HLS INTERFACE s_axilite port=dout_gmem bundle=ctrl_reg offset=0x040

	// DDR memory Interface
#pragma HLS INTERFACE m_axi port=d_ddrmem bundle=card_mem0 offset=slave depth=512 \
  max_read_burst_length=64  max_write_burst_length=64 
#pragma HLS INTERFACE s_axilite port=d_ddrmem bundle=ctrl_reg offset=0x050

	// Host Memory AXI Lite Master Interface
#pragma HLS DATA_PACK variable=act_reg
#pragma HLS INTERFACE s_axilite port=act_reg bundle=ctrl_reg offset=0x100
#pragma HLS INTERFACE s_axilite port=return bundle=ctrl_reg

        process_action(din_gmem, dout_gmem, d_ddrmem, act_reg);

        act_reg->Control.Retc = SNAP_RETC_SUCCESS;
}
//-----------------------------------------------------------------------------
//--- TESTBENCH ---------------------------------------------------------------
//-----------------------------------------------------------------------------

#ifdef NO_SYNTH

int main(void)
{
	#define MEMORY_LINES_512  1024 /* 64 KiB */
	#define MEMORY_LINES_1024 512  /* 64 KiB */

	static snap_membus_1024_t  din_gmem[TOTAL_MEMORY_LINES];
	static snap_membus_1024_t  dout_gmem[TOTAL_MEMORY_LINES];
	static snap_membus_512_t   d_ddrmem[MEMORY_LINES_512];
	#ifdef DEBUG
		printf("TOTAL_MEMORY_LINES %d\n",TOTAL_MEMORY_LINES);
	#endif
	int rc = 0;
	int test=0;
	printf("GOps %f\n",float(1.0E-09*50*(GRIDROWS-4)*(GRIDCOLUMNS-4)*(GRIDDEPTH)));
	printf("Grid %dx%dx%d\n",GRIDROWS,GRIDCOLUMNS,GRIDDEPTH);
	printf("window %dx%d\n",ROWS,COLUMNS);
	unsigned int i;
	action_reg act_reg;


	#ifdef DEBUG
		printf("sizeof(mat_elmt_t %d\n",sizeof(mat_elmt_t));
		printf("\nsizeof(din_gmemm %d\n",sizeof(din_gmem));

		printf("\nNumber of elements in gmem %d\n",sizeof(din_gmem)/sizeof(din_gmem[0]));
	#endif
	memset(din_gmem,  0xA, sizeof(din_gmem));
	memset(dout_gmem, 0xB, sizeof(dout_gmem));
	memset(d_ddrmem,  0xC, sizeof(d_ddrmem));
	
	union gg{
		mat_elmt_t   value_d; //float
		uint64_t     value_u;
	};

	printf(HLINE);
	printf("****************INITIALIZING***************************\n");
	gg gg1;
	//Fill a table with 16 values: 1.0, 1.5, 2.0, 2.5,... 8.5
	//Not fullly debugged yet
	int v=0;


	#ifdef TEST
		static mat_elmt_t test_in[GRIDDEPTH][GRIDCOLUMNS][GRIDROWS];
		static mat_elmt_t test_out[GRIDDEPTH][GRIDCOLUMNS][GRIDROWS];
		printf(HLINE);
		printf("REFERENCE REQUESTED\n");
		printf(HLINE);

		memset(test_out,0 , sizeof(test_out));
		memset(test_in,0 , sizeof(test_in));
		for (int d=0;d<GRIDDEPTH;d++)
		{
			for(int c=0;c<GRIDCOLUMNS;c++){
			//#pragma HLS PIPELINE II=2
				for(int r=0;r<GRIDROWS;r++){
					test_in[d][c][r]=1+0.7*v;
					#ifdef TESTALLOC
						printf("DEPTH,ROW,COLUMN: %d,%d,%d",d,c,r);
						//printf(":::assign::::%f\n",value_d);

						printf("eh krti assign::::%f\n",test_in[d][c][r]);
					#endif
					v++;

				}
			}
		}
		//printf("FINAL V DI VALUE=%d",v);
		for (int d=0;d<GRIDDEPTH;d++)
		{
			loop_stencilColumn:for(int c=2;c<GRIDCOLUMNS-2;c++){
				//#pragma HLS PIPELINE II=2

				loop_stencilRow:for(int r=2;r<GRIDROWS-2;r++){

					double lap_ij = 4 *test_in[d][c][r] - test_in[d][c][r-1] - test_in[d][c][r+1] -
					test_in[d][c-1][r] - test_in[d][c+1][r];

					double lap_imj =  4 *test_in[d][c][r-1] - test_in[d][c][r-2] - test_in[d][c][r] -
					test_in[d][c-1][r-1] - test_in[d][c+1][r-1];

					double lap_ipj = 4 *test_in[d][c][r+1] - test_in[d][c][r] - test_in[d][c][r+2] -
					test_in[d][c-1][r+1] - test_in[d][c+1][r+1];

					double lap_ijm =  4 *test_in[d][c-1][r] - test_in[d][c-1][r-1] - test_in[d][c-1][r+1] -
					test_in[d][c-2][r] - test_in[d][c][r];


					double lap_ijp =  4 *test_in[d][c+1][r] - test_in[d][c+1][r-1] - test_in[d][c+1][r+1] -
					test_in[d][c][r] - test_in[d][c+2][r];


					double flx_ij = lap_ipj - lap_ij;
					flx_ij = flx_ij * (test_in[d][c][r+1] - test_in[d][c][r]) > 0 ? 0 : flx_ij;

					double flx_imj = lap_ij - lap_imj;
					flx_imj = flx_imj * (test_in[d][c][r] - test_in[d][c][r-1]) > 0 ? 0 : flx_imj;

					double fly_ij = lap_ijp - lap_ij;
					fly_ij = fly_ij * (test_in[d][c+1][r] - test_in[d][c][r]) > 0 ? 0 : fly_ij;

					double fly_ijm = lap_ij - lap_ijm;
					fly_ijm = fly_ijm * (test_in[d][c][r] - test_in[d][c-1][r]) > 0 ? 0 : fly_ijm;

					test_out[d][c][r] = test_in[d][c][r] - 7 * (flx_ij - flx_imj + fly_ij - fly_ijm);

					#ifdef TESTRESULT
						printf("REFERENCE[%d,%d,%d]::::%f\n",d,c,r,test_out[d][c][r]);

					#endif
				}
			}
		}


		printf(HLINE);
		printf(HLINE);

	#endif

	v=0;
	ALLOCL1:for (int i=0;i<TOTAL_MEMORY_LINES;i++)
	{
		#ifdef DEBUG
			printf(HLINE);
			printf("MEMORY line:::%d\n",i);
			//printf("MEMORY line:::%d\n",i);
		#endif
		ALLOCL2:	for(int j = 0; j < 32 ; j++){
				gg1.value_d = 1+0.7*v;

				//gg1.value_d = temp_ptr+i*32+j;
				int high=(8*sizeof(mat_elmt_t)*(j+1))-1;
				int low=8*sizeof(mat_elmt_t)*j;
				din_gmem[i](high,low)=gg1.value_u;

				//printf("\t***din_gmem(%d,%d)*****%f\n",high,low,gg1.value_d);
				#ifdef DEBUG
					printf("\t***din_gmem(%d,%d)*****%lf\n",high,low,gg1.value_d);
				#endif
				v++;

			}

	}
//printf("FINAL V DI VALUE=%d",v);
printf("*****************INTIALIZATION DONE***************************\n");
printf(HLINE);
    act_reg.Control.flags = 0x1; /* just not 0x0 */

    act_reg.Data.in.addr = 0;
    act_reg.Data.in.size = GRIDDEPTH*GRIDCOLUMNS*GRIDROWS*sizeof(float);
    act_reg.Data.in.type = SNAP_ADDRTYPE_HOST_DRAM;

    act_reg.Data.out.addr = 4096;
    act_reg.Data.out.size = GRIDDEPTH*GRIDCOLUMNS*GRIDROWS*sizeof(float);
    act_reg.Data.out.type = SNAP_ADDRTYPE_HOST_DRAM;

    hls_action(din_gmem, dout_gmem, d_ddrmem, &act_reg);
    if (act_reg.Control.Retc == SNAP_RETC_FAILURE) {
	    fprintf(stderr, " ==> RETURN CODE FAILURE <==\n");
	    return 1;
    }

      printf(HLINE);
	printf("****************BACK AGAIN IN MAIN PROGRAM***************************\n");

#ifdef TEST
	printf("****************RECEIVED STENCIL**************************\n");
  //  test_bus_to_grid3D(dout_gmem,bufferOut, data_out,o_idx, 0x0,0,TOTAL_MEMORY_LINES);
#endif

//
   	printf("*****************THE END**************************\n");
   	printf(HLINE);
//
//
    return 0;
}

#endif
