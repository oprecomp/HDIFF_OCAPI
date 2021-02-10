# HDIFF_OCAPI
OpenCAPI FPGA accelerator for High Order Stencils (HOL)


* This repository includes the implementation of an FPGA accelerator for a High Order Stencil (HOL), specifically the Horizontal Diffusion  (HDIFF) from [COSMO whether model](http://www.cosmo-model.org/).
* The accelerator has been implemented using the [OC-Accel Transprecision Framework](https://github.com/oprecomp/oc-accel-tp).
* C code computes HDIFF by moving data from the CPU host memory to FPGA over OCAPI
* There is an option to have multiple parallel accelerators. We do procession in ROW and COLUMN while the DEPTH remains constant.


# Cite
* G. Singh, D. Diamantopoulos, C. Hagleitner, S. Stuijk and H. Corporaal, "NARMADA: Near-Memory Horizontal Diffusion Accelerator for Scalable Stencil Computations," 2019 29th International Conference on Field Programmable Logic and Applications (FPL), Barcelona, Spain, 2019, pp. 263-269, doi: 10.1109/FPL.2019.00050.
* G. Singh, D. Diamantopoulos, S. Stuijk, C. Hagleitner, H. Corporaal, "Low Precision Processing for High Order Stencil Computations", SAMOS 2019, pp. 403-415

# Acknowledgement
* The initial software code was provided by [CSCS](https://www.cscs.ch/), under the course of [NeMeCo project](https://cordis.europa.eu/project/id/676240) and Ronald P. Luijten' Sabbatical at ETH.
* The novel HW-SW co-design using Oc-Accel framework was done by Gagandeep Singh, during his research visit at IBM Zurich lab.
* This work was funded by the European Union’s H2020 research and innovation program under grant agreement No 732631, project OPRECOMP. For details visit http://oprecomp.eu/.

# License
The source code in this repository is licensed under the terms of the Apache License (Version 2.0), unless otherwise specified. See LICENSE for details.
