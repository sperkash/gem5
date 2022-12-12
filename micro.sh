#!/bin/bash
#
#SBATCH --cpus-per-task=8
#SBATCH --time=60:00
#SBATCH --mem=30G

## Max CPUs: 8
## Max time: 60 min
## Max mem: 40 GBs

source /data/.local/modules/init/bash
export REPO=/localhome/fubof/security_final_project/InvisiSpec-1.0;
export GEM5_PATH=$REPO;
export SPEC_SRC_PATH=/data/benchmarks/spec2017
export SPEC_BASE_DIR=/data/benchmarks/spec2017/benchspec/CPU/; # Add $BENCHNUM.$BENCH/exe/$BENCH_base.mytest-m64

module load llvm-11
cd $REPO

scons build/X86_MESI_Two_Level/gem5.opt -j 97

# # Actually launch gem5!
# $GEM5_PATH/build/X86_MESI_Two_Level/gem5.fast \
# 	--outdir=$OUTPUT_DIR $GEM5_PATH/configs/example/spec06_config.py \
# 	--benchmark=$BENCHMARK --benchmark_stdout=$OUTPUT_DIR/$BENCHMARK.out \
# 	--benchmark_stderr=$OUTPUT_DIR/$BENCHMARK.err \
# 	--num-cpus=1 --mem-size=4GB \
# 	--checkpoint-dir=$CKPT_OUT_DIR \
# 	--checkpoint-restore=10000000000 --at-instruction \
#     --l1d_assoc=8 --l2_assoc=16 --l1i_assoc=4 \
#     --cpu-type=DerivO3CPU --needsTSO=0 --scheme=$SCHEME \
#     --num-dirs=1 --ruby --maxinsts=2000000000 \
#     --network=simple --topology=Mesh_XY --mesh-rows=1 | tee -a $SCRIPT_OUT


# $M5_PATH/build/ARM/gem5.opt --debug-flags=HWACC,DeviceMMR,LLVMInterface,AddrRanges,NoncoherentDma,RuntimeCompute --outdir=$OUTDIR $REPO/gem5-config/fs_${smallACC}.py --mem-size=8GB --kernel=$REPO/dma/$smallACC/host/main.elf --disk-image=$M5_PATH/baremetal/common/fake.iso --machine-type=VExpress_GEM5_V1 --dtb-file=none --bare-metal --cpu-type=DerivO3CPU --accpath=$REPO/dma --accbench=$smallACC --caches --l2cache --acc_cache --input $MNIST_INPUT_GEM5 --m0 inputs/mnist/bin/m0.bin --m1 inputs/mnist/bin/m1.bin > $OUTDIR/debug-trace.txt