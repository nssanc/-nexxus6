#
# This is a modified version of zlib, which does all memory
# allocation ahead of time.
#
# This is the compression code, see zlib_inflate for the
# decompression code.
#
CFLAGS_KERNEL := -fno-graphite \
          	-fno-graphite-identity \
          	-fno-loop-flatten \
          	-fno-tree-loop-linear \
          	-fno-loop-interchange \
          	-fno-loop-strip-mine \
          	-fno-loop-block \
          	-fno-loop-nest-optimize \
          	-fno-loop-unroll-and-jam \
          	-fno-loop-parallelize-all \
          	-ftree-parallelize-loops=0 \
          	-fno-openmp

obj-$(CONFIG_ZLIB_DEFLATE) += zlib_deflate.o

zlib_deflate-objs := deflate.o deftree.o deflate_syms.o
