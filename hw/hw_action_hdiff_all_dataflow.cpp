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
//#define TESTALLOC
//#define TESTRESULT
//#define DEBUG


// WRITE DATA TO MEMORY
short write_burst_of_data_to_mem(snap_membus_1024_t *dout_gmem,
				 snap_membus_512_t *d_ddrmem,
				 snapu16_t memory_in_type,
				 snapu16_t memory_out_type,
				 snapu64_t output_address_1024,
				 snapu64_t output_address,
				 snap_membus_1024_t *buffer_gmem,
				 snap_membus_512_t *buffer_ddrmem,
				 snapu64_t size_in_bytes_to_transfer)
{
	short rc;
    ap_int<MEMDW_512> mask_full = -1;
    snap_membus_1024_t mask_512 = snap_membus_512_t(mask_full);
    snap_membus_1024_t data_entry = 0;

	// Prepare Patch to the issue#652 - memcopy doesn't handle small packets
	int size_in_words_1024;
	if(size_in_bytes_to_transfer %BPERDW_1024 == 0)
		size_in_words_1024 = size_in_bytes_to_transfer/BPERDW_1024;
	else
		size_in_words_1024 = (size_in_bytes_to_transfer/BPERDW_1024) + 1;

	int size_in_words_512;
	if(size_in_bytes_to_transfer %BPERDW_512 == 0)
		size_in_words_512 = size_in_bytes_to_transfer/BPERDW_512;
	else
		size_in_words_512 = (size_in_bytes_to_transfer/BPERDW_512) + 1;
	//end of patch
//========================data from buffer_gmem======================================//
	if (memory_in_type == SNAP_ADDRTYPE_HOST_DRAM) {
           if(memory_out_type == SNAP_ADDRTYPE_HOST_DRAM) {
		      // Patch to the issue#652 - memcopy doesn't handle small packets
		      // Do not insert anything more in this loop to not break the burst
		      wb_dout2dout_loop: for (int k=0; k<size_in_words_1024; k++)
		      #pragma HLS PIPELINE
                          (dout_gmem + output_address_1024)[k] = buffer_gmem[k];
		      // end of patch
       	      rc = 0;
           }
           else if(memory_out_type == SNAP_ADDRTYPE_CARD_DRAM) {
		     wb_gbuf2dbuf_loop: for (int k=0; k<size_in_words_1024; k++) {
                                  for (int j=0; j<MEMDW_1024/MEMDW_512; j++) {
		     #pragma HLS PIPELINE
                                    buffer_ddrmem[k*MEMDW_1024/MEMDW_512+j] = (snap_membus_512_t)((buffer_gmem[k] >> j*MEMDW_512) & mask_512);
                                  }
                               }
		     wb_dbuf2ddr_loop: for (int k=0; k<size_in_words_512; k++)
		     #pragma HLS PIPELINE
                         (d_ddrmem + output_address)[k] = buffer_ddrmem[k];
             rc = 0;
           }
           else if(memory_out_type == SNAP_ADDRTYPE_UNUSED)
             rc = 0;
           else
             rc = 1;
	}
//=========================data from buffer_ddrmem=====================================//
	else if (memory_in_type == SNAP_ADDRTYPE_CARD_DRAM) {
           if(memory_out_type == SNAP_ADDRTYPE_HOST_DRAM) {
		     wb_dbuf2gbuf_loop: for (int k=0; k<size_in_words_1024; k++) {
                                  for (int j=0; j<MEMDW_1024/MEMDW_512; j++) {
		     #pragma HLS PIPELINE
                                    data_entry |= ((snap_membus_1024_t)(buffer_ddrmem[k*MEMDW_1024/MEMDW_512+j])) << j*MEMDW_512;
                                  }
                                  buffer_gmem[k] = data_entry;
                                  data_entry = 0;
                                }
		     wb_gbuf2dout_loop: for (int k=0; k<size_in_words_1024; k++)
		     #pragma HLS PIPELINE
                         (dout_gmem + output_address_1024)[k] = buffer_gmem[k];
             rc = 0;
           }
           else if(memory_out_type == SNAP_ADDRTYPE_CARD_DRAM) {
		     wb_ddr2ddr_loop: for (int k=0; k<size_in_words_512; k++)
		     #pragma HLS PIPELINE
                        (d_ddrmem + output_address)[k] = buffer_ddrmem[k];
             rc = 0;
           }
           else if(memory_out_type == SNAP_ADDRTYPE_UNUSED)
             rc = 0;
           else
             rc = 1;
	}
//========================no data from specified=======================================//
	else if (memory_in_type == SNAP_ADDRTYPE_UNUSED) {
           if(memory_out_type == SNAP_ADDRTYPE_HOST_DRAM) {
		      wb_dout_loop: for (int k=0; k<size_in_words_1024; k++)
		      #pragma HLS PIPELINE
                          (dout_gmem + output_address_1024)[k] = buffer_gmem[k];
       	      rc = 0;
           }
           else if(memory_out_type == SNAP_ADDRTYPE_CARD_DRAM) {
		     wb_ddr_loop: for (int k=0; k<size_in_words_512; k++)
		     #pragma HLS PIPELINE
                        (d_ddrmem + output_address)[k] = buffer_ddrmem[k];
             rc = 0;
           }
           else if(memory_out_type == SNAP_ADDRTYPE_UNUSED)
             rc = 0;
           else
             rc = 1;
    }
//=====================================================================================//
	else
		rc = 1;

	return rc;
}

// READ DATA FROM MEMORY
short read_burst_of_data_from_mem(snap_membus_1024_t *din_gmem,
				  snap_membus_512_t *d_ddrmem,
				  snapu16_t memory_type,
				  snapu64_t input_address_1024,
				  snapu64_t input_address,
				  snap_membus_1024_t *buffer_gmem,
				  snap_membus_512_t *buffer_ddrmem,
				  snapu64_t size_in_bytes_to_transfer)
{
	short rc;
        int i;

	switch (memory_type) {

	case SNAP_ADDRTYPE_HOST_DRAM:
		memcpy(buffer_gmem, (snap_membus_1024_t  *) (din_gmem + input_address_1024),
		       size_in_bytes_to_transfer);
       		rc =  0;
		break;
	case SNAP_ADDRTYPE_CARD_DRAM:
		memcpy(buffer_ddrmem, (snap_membus_512_t  *) (d_ddrmem + input_address),
		       size_in_bytes_to_transfer);
       		rc =  0;
		break;
	case SNAP_ADDRTYPE_UNUSED: /* no copy but with rc =0 */
       		rc =  0;
		break;
	default:
		rc = 1;
	}

	return rc;
}



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
			//#pragma HLS PIPELINE
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
	snap_membus_1024_t* temp_ptr = din_gmem + input_address;
	rd_din_loop: for (unsigned k=0; k < size_in_lines; k++) {
		//printf("size_in lines=%d\n",k);
	#pragma HLS PIPELINE
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
		// #pragma HLS UNROLL

			//printf("stream_splitter::ACCELERATOR:::%d\n",p);
			for (unsigned i = 0; i < lines; i ++)
			{
			//	printf("stream_splitter:::::%d\n",i);
				#pragma HLS LOOP_TRIPCOUNT max=lines 
				temp = output[p].read();
				//  #pragma HLS LOOP_TRIPCOUNT max=lines avg=lines/2
				#pragma HLS PIPELINE II=1
				bus_output.write(temp);
			}
		}
	}
}

void stream_2D_grid(mat_elmt_t hdiff_in[COLUMNS][ROWS],unsigned acc,
		hls::stream<snap_membus_1024_t> &dataTable,snapu64_t size_in_lines) {
#pragma HLS INLINE OFF
	  mat_elmt_t d=0;
	  mat_elmt_t* temp_ptr = &hdiff_in[0][0];
	  union {
		uint64_t     value_u;
		mat_elmt_t   value_d;
	};
  selection_loop: for (unsigned i = 0; i < size_in_lines; i++) {
  #pragma HLS PIPELINE II=1
  	//printf("steam_2D_grid:::cacheline=%d \n",i);
    	snap_membus_1024_t tempRead=dataTable.read();
    	HistogramFPGAStream_label2: for(int j = 0; j < 32; j++)
    	{
	#pragma HLS UNROLL
     		value_u=tempRead.range((8*sizeof(mat_elmt_t)*(j+1))-1, (8*sizeof(mat_elmt_t)*j));
        		*(temp_ptr + ( i *32) + j)=value_d;



        		#ifdef TESTALLOC
			printf("GRIDDING:::[%d]::::%f\n",( i *32) + j,value_d);
		#endif


    	}
   

  	  }
}


void grid_2D_stream(mat_elmt_t hdiff_out[COLUMNS][ROWS],unsigned acc,
		hls::stream<snap_membus_1024_t> &data_out,snapu64_t size_in_lines) {
	#pragma HLS INLINE OFF
	mat_elmt_t d=0;

	mat_elmt_t* temp_ptr = &hdiff_out[0][0];
	union {
	uint64_t     value_u;
	mat_elmt_t   value_d;
	};
	snap_membus_1024_t temp=0;
	selection_loop: for (unsigned i = 0; i < size_in_lines; i++) {
	{
		#pragma HLS PIPELINE II=1
		HistogramFPGAStream_label2: for(int j = 0; j < 32; j++)
		{
			#pragma HLS UNROLL
			value_d=*(temp_ptr + ( i *32) + j);
			temp.range((8*sizeof(mat_elmt_t)*(j+1))-1, (8*sizeof(mat_elmt_t)*j))=value_u;
			//printf("SENDING DATA d[%d]=%f\n",i*32+j,*(temp_ptr + ( i *32) + j));

		}
	}
	data_out.write(temp);
	}
}
// WRITE DATA TO MEMORY
void write_stream_to_mem (hls::stream<snap_membus_1024_t>& out_stream,
						snap_membus_1024_t *dout_gmem,
						snapu64_t output_address,
						snapu64_t size_in_lines)
{
	snap_membus_1024_t* temp_ptr = dout_gmem + output_address;
	wb_dout_loop: for (unsigned k=0; k < size_in_lines; k++) {
#pragma HLS PIPELINE
		temp_ptr[k] = out_stream.read();
	}
}



void real_computation( hls::stream<snap_membus_1024_t> &in_stream,
	hls::stream<snap_membus_1024_t> &out_stream,unsigned ac)
{	
	#pragma HLS INLINE OFF
	const int row=ROWS;
	const int col=COLUMNS;
	mat_elmt_t hdiff_in[COLUMNS][ROWS];
 	//#pragma HLS ARRAY_PARTITION variable=hdiff_in complete dim=1
  	#pragma HLS STREAM variable=hdiff_in depth=row*col
  	mat_elmt_t hdiff_out[COLUMNS][ROWS];
 	//#pragma HLS ARRAY_PARTITION variable=hdiff_in complete dim=1
  	#pragma HLS STREAM variable=hdiff_out depth=row*col
	#pragma HLS DATAFLOW
	   	  stream_2D_grid(hdiff_in,ac, in_stream,MAX_NB_OF_LINES_READ);
	   	  hdiff_2D_stream(hdiff_in,hdiff_out);
	   	  grid_2D_stream(hdiff_out,ac,out_stream,MAX_NB_OF_LINES_READ) ;


}

void fpga_execution( hls::stream<snap_membus_1024_t> in_stream[MANY_ACC],hls::stream<snap_membus_1024_t> out_stream[MANY_ACC])
{	
#pragma HLS INLINE OFF

	const int acc=MANY_ACC;
	int calls_per_acc=TOTAL_MEMORY_LINES/MAX_NB_OF_LINES_READ/acc;
	printf("calls_per_acc=%d\n",calls_per_acc);
	for (unsigned c = 0; c < calls_per_acc; c++)
	{
	  for(unsigned ac=0;ac<acc;ac++)
	   {
	   #pragma HLS UNROLL complete
	   printf("computation\n");
	   
	  real_computation(in_stream[ac],out_stream[ac],ac);
	   	// calls_per_acc--;
	   }
	  }

}


void hdiff_in (snap_membus_1024_t *din_gmem, 
		snap_membus_1024_t *dout_gmem, 
		snapu64_t input_address,
		snapu64_t output_address,
		snapu64_t size_in_lines)
{
	#pragma HLS INLINE OFF
	#pragma HLS DATAFLOW
	const int acc=MANY_ACC;
	hls::stream<snap_membus_1024_t> bus_in_stream("bus_in_stream");
	hls::stream<snap_membus_1024_t> bus_out_stream("bus_out_stream");

	hls::stream<snap_membus_1024_t> in_stream[acc];
	hls::stream<snap_membus_1024_t> out_stream[acc];
	
	read_mem_to_stream(bus_in_stream, din_gmem, input_address, TOTAL_MEMORY_LINES);
	stream_splitter (bus_in_stream,in_stream, MAX_NB_OF_LINES_READ);
	fpga_execution(in_stream,out_stream);
	stream_merger (bus_out_stream,out_stream,MAX_NB_OF_LINES_READ);
	write_stream_to_mem(bus_out_stream, dout_gmem, output_address, TOTAL_MEMORY_LINES);


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
	snap_membus_1024_t  buf_gmem[MAX_NB_OF_WORDS_READ_1024];
	snap_membus_512_t   buf_ddrmem[MAX_NB_OF_WORDS_READ_512];
	// if 4096 bytes max => 64 words

	// byte address received need to be aligned with port width
	InputAddress_1024 = (act_reg->Data.in.addr)   >> ADDR_RIGHT_SHIFT_1024;
	OutputAddress_1024 = (act_reg->Data.out.addr) >> ADDR_RIGHT_SHIFT_1024;
	InputAddress_512 = (act_reg->Data.in.addr)   >> ADDR_RIGHT_SHIFT_512;
	OutputAddress_512 = (act_reg->Data.out.addr) >> ADDR_RIGHT_SHIFT_512;

	address_xfer_offset_512 = 0x0;
	address_xfer_offset_1024 = 0x0;
	// testing sizes to prevent from writing out of bounds
	action_xfer_size = MIN(act_reg->Data.in.size,
			       act_reg->Data.out.size);
	bool fpgaMem=false;
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

	// buffer size is hardware limited by MAX_NB_OF_BYTES_READ
	if(action_xfer_size %MAX_NB_OF_BYTES_READ == 0)
		nb_blocks_to_xfer = (action_xfer_size / MAX_NB_OF_BYTES_READ);
	else
		nb_blocks_to_xfer = (action_xfer_size / MAX_NB_OF_BYTES_READ) + 1;
	printf("Inside action\n");
	uint32_t size;
	int kernelCall=TOTAL_MEMORY_LINES/MAX_NB_OF_LINES_READ;
	printf("kernelCall %d\n",kernelCall);

const int acc=MANY_ACC;

//  mat_elmt_t data_out[MANY_ACC][COLUMNS][ROWS];

// #pragma HLS ARRAY_PARTITION variable=data_out complete dim=1

 hdiff_in (din_gmem, 
	dout_gmem,InputAddress_1024,OutputAddress_1024,
	MAX_NB_OF_LINES_READ);


	if (rc != 0)
		ReturnCode = SNAP_RETC_FAILURE;

	act_reg->Control.Retc = ReturnCode;
	return;
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
    act_reg.Data.in.size = MAX_NB_OF_LINES_READ;
    act_reg.Data.in.type = SNAP_ADDRTYPE_HOST_DRAM;

    act_reg.Data.out.addr = 4096;
    act_reg.Data.out.size = MAX_NB_OF_LINES_READ;
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
