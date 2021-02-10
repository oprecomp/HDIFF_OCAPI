ulimit -s unlimited 
g++ -g -fopenmp -o main -DNO_SYNTH -I../../include -I../../../software/include  -I../include -I$XILINX_VIVADO/include  hw_action_hdiff.cpp