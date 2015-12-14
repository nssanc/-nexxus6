#
# Makefile for the packet AF.
#
CFLAGS_KERNEL := -Wno-misleading-indentation

obj-$(CONFIG_PACKET) += af_packet.o
obj-$(CONFIG_PACKET_DIAG) += af_packet_diag.o
af_packet_diag-y += diag.o
