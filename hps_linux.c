#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>
#include <time.h>
#include "alt_generalpurpose_io.h"
#include <hwlib.h>
#include <socal/alt_gpio.h>
#include <socal/hps.h>
#include <socal/socal.h>
#include "functions/avalon_spi.h"
#include "hps_linux.h"
#include "./soc_variables/soc_system.h"
#include "./soc_variables/general.h"
#include "./soc_variables/lm96570_vars.h"
#include "./soc_variables/ad9276_vars.h"
#include "functions/AlteraIP/altera_avalon_fifo_regs.h"

// parameters
unsigned int num_of_samples = 8000;
const unsigned int num_of_switches = 11;
const unsigned int num_of_channels = 8;
extern int fd_dev_mem;

unsigned int cnt_out_val;
void create_measurement_folder(char * foldertype) {
	time_t t = time(NULL);
	struct tm tm = *localtime(&t);
	char command[60];
	sprintf(foldername, "%s_%04d_%02d_%02d_%02d_%02d_%02d", foldertype, tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
	sprintf(command, "mkdir %s", foldername);
	system(command);
// copy the executable file to the folder
	sprintf(command, "cp ./thesis_nmr_de1soc_hdl2.0 %s/execfile", foldername);
	system(command);
}
void init() {

	//printf("ULTRASOUND SYSTEM STARTS!\n");
	// open device memory
	fd_dev_mem = open("/dev/mem", O_RDWR | O_SYNC);
	if (fd_dev_mem == -1) {
		printf("ERROR: could not open \"/dev/mem\".\n");
		printf(" errno = %s\n", strerror(errno));
		exit (EXIT_FAILURE);
	}

	// mmap hps peripherals
	hps_gpio = mmap(NULL, hps_gpio_span, PROT_READ | PROT_WRITE, MAP_SHARED, fd_dev_mem, hps_gpio_ofst);
	if (hps_gpio == MAP_FAILED) {
		printf("Error: hps_gpio mmap() failed.\n");
		printf(" errno = %s\n", strerror(errno));
		close(fd_dev_mem);
		exit (EXIT_FAILURE);
	}

	// mmap fpga peripherals
	h2f_lw_axi_master = mmap(NULL, h2f_lw_axi_master_span, PROT_READ | PROT_WRITE, MAP_SHARED, fd_dev_mem, h2f_lw_axi_master_ofst);
	if (h2f_lw_axi_master == MAP_FAILED) {
		printf("Error: h2f_lw_axi_master mmap() failed.\n");

		printf(" errno = %s\n", strerror(errno));
		close(fd_dev_mem);
		exit (EXIT_FAILURE);
	}
	// h2p_fifo_sink_csr_addr = h2f_lw_axi_master + FIFO_SINK_OUT_CSR_BASE;
	// h2p_fifo_sink_data_addr = h2f_lw_axi_master + FIFO_SINK_OUT_BASE;
	h2p_fifo_sink_ch_a_csr_addr = h2f_lw_axi_master + FIFO_SINK_CH_A_OUT_CSR_BASE;
	h2p_fifo_sink_ch_a_data_addr = h2f_lw_axi_master + FIFO_SINK_CH_A_OUT_BASE;
	h2p_fifo_sink_ch_b_csr_addr = h2f_lw_axi_master + FIFO_SINK_CH_B_OUT_CSR_BASE;
	h2p_fifo_sink_ch_b_data_addr = h2f_lw_axi_master + FIFO_SINK_CH_B_OUT_BASE;
	h2p_fifo_sink_ch_c_csr_addr = h2f_lw_axi_master + FIFO_SINK_CH_C_OUT_CSR_BASE;
	h2p_fifo_sink_ch_c_data_addr = h2f_lw_axi_master + FIFO_SINK_CH_C_OUT_BASE;
	h2p_fifo_sink_ch_d_csr_addr = h2f_lw_axi_master + FIFO_SINK_CH_D_OUT_CSR_BASE;
	h2p_fifo_sink_ch_d_data_addr = h2f_lw_axi_master + FIFO_SINK_CH_D_OUT_BASE;
	h2p_fifo_sink_ch_e_csr_addr = h2f_lw_axi_master + FIFO_SINK_CH_E_OUT_CSR_BASE;
	h2p_fifo_sink_ch_e_data_addr = h2f_lw_axi_master + FIFO_SINK_CH_E_OUT_BASE;
	h2p_fifo_sink_ch_f_csr_addr = h2f_lw_axi_master + FIFO_SINK_CH_F_OUT_CSR_BASE;
	h2p_fifo_sink_ch_f_data_addr = h2f_lw_axi_master + FIFO_SINK_CH_F_OUT_BASE;
	h2p_fifo_sink_ch_g_csr_addr = h2f_lw_axi_master + FIFO_SINK_CH_G_OUT_CSR_BASE;
	h2p_fifo_sink_ch_g_data_addr = h2f_lw_axi_master + FIFO_SINK_CH_G_OUT_BASE;
	h2p_fifo_sink_ch_h_csr_addr = h2f_lw_axi_master + FIFO_SINK_CH_H_OUT_CSR_BASE;
	h2p_fifo_sink_ch_h_data_addr = h2f_lw_axi_master + FIFO_SINK_CH_H_OUT_BASE;
	h2p_led_addr = h2f_lw_axi_master + LED_PIO_BASE;
	h2p_sw_addr = h2f_lw_axi_master + DIPSW_PIO_BASE;

	h2p_button_addr = h2f_lw_axi_master + BUTTON_PIO_BASE;
	h2p_adcspi_addr = h2f_lw_axi_master + AD9276_SPI_BASE;
	h2p_adc_samples_per_echo_addr = h2f_lw_axi_master + ADC_SAMPLES_PER_ECHO_BASE;
	h2p_init_delay_addr = h2f_lw_axi_master + ADC_INIT_DELAY_BASE;
	h2p_spi_num_of_bits_addr = h2f_lw_axi_master + LM96570_SPI_NUM_OF_BITS_BASE;
	h2p_general_cnt_int_addr = h2f_lw_axi_master + GENERAL_CNT_IN_BASE;
	h2p_general_cnt_out_addr = h2f_lw_axi_master + GENERAL_CNT_OUT_BASE;
	h2p_lm96570_spi_out2_addr = h2f_lw_axi_master + LM96570_SPI_OUT_2_BASE;
	h2p_lm96570_spi_out1_addr = h2f_lw_axi_master + LM96570_SPI_OUT_1_BASE;
	h2p_lm96570_spi_out0_addr = h2f_lw_axi_master + LM96570_SPI_OUT_0_BASE;
	h2p_lm96570_spi_in2_addr = h2f_lw_axi_master + LM96570_SPI_IN_2_BASE;
	h2p_lm96570_spi_in1_addr = h2f_lw_axi_master + LM96570_SPI_IN_1_BASE;
	h2p_lm96570_spi_in0_addr = h2f_lw_axi_master + LM96570_SPI_IN_0_BASE;
	h2p_adc_start_pulselength_addr = h2f_lw_axi_master + ADC_START_PULSELENGTH_BASE;
	h2p_mux_control_addr = h2f_lw_axi_master + MUX_CONTROL_BASE;
// write default value for cnt_out
	cnt_out_val = CNT_OUT_DEFAULT;
	alt_write_word((h2p_general_cnt_out_addr), cnt_out_val);
}
void leave() {
	close(fd_dev_mem);
	// munmap hps peripherals
	if (munmap(hps_gpio, hps_gpio_span) != 0) {
		printf("Error: hps_gpio munmap() failed\n");
		printf(" errno = %s\n", strerror(errno));
		close(fd_dev_mem);

		exit (EXIT_FAILURE);
	}
	hps_gpio = NULL;
	if (munmap(h2f_lw_axi_master, h2f_lw_axi_master_span) != 0) {
		printf("Error: h2f_lw_axi_master munmap() failed\n");
		printf(" errno = %s\n", strerror(errno));
		close(fd_dev_mem);
		exit (EXIT_FAILURE);
	}
	h2f_lw_axi_master = NULL;
	h2p_led_addr = NULL;
	h2p_sw_addr = NULL;
	h2p_fifo_sink_ch_a_csr_addr = NULL;
	h2p_fifo_sink_ch_a_data_addr = NULL;
	h2p_fifo_sink_ch_b_csr_addr = NULL;
	h2p_fifo_sink_ch_b_data_addr = NULL;
	h2p_fifo_sink_ch_c_csr_addr = NULL;
	h2p_fifo_sink_ch_d_data_addr = NULL;
	h2p_fifo_sink_ch_d_csr_addr = NULL;
	h2p_fifo_sink_ch_d_data_addr = NULL;
	h2p_fifo_sink_ch_e_csr_addr = NULL;
	h2p_fifo_sink_ch_e_data_addr = NULL;
	h2p_fifo_sink_ch_f_csr_addr = NULL;
	h2p_fifo_sink_ch_f_data_addr = NULL;
	h2p_fifo_sink_ch_g_csr_addr = NULL;
	h2p_fifo_sink_ch_g_data_addr = NULL;
	h2p_fifo_sink_ch_h_csr_addr = NULL;
	h2p_fifo_sink_ch_h_data_addr = NULL;
	h2p_led_addr = NULL;
	h2p_sw_addr = NULL;
	h2p_button_addr = NULL;
	h2p_adcspi_addr = NULL;
	h2p_adc_samples_per_echo_addr = NULL;
	h2p_init_delay_addr = NULL;
	h2p_spi_num_of_bits_addr = NULL;
	h2p_general_cnt_int_addr = NULL;
	h2p_general_cnt_out_addr = NULL;
	h2p_lm96570_spi_out2_addr = NULL;
	h2p_lm96570_spi_out1_addr = NULL;

	h2p_lm96570_spi_out0_addr = NULL;
	h2p_lm96570_spi_in2_addr = NULL;
	h2p_lm96570_spi_in1_addr = NULL;
	h2p_lm96570_spi_in0_addr = NULL;
	h2p_mux_control_addr = NULL;
	//printf("\nULTRASOUND SYSTEM STOPS!\n");
}

unsigned int write_adc_spi(unsigned int comm) {
	unsigned int data;
	while (!(alt_read_word(h2p_adcspi_addr + SPI_STATUS_offst) & (1 << status_TRDY_bit)))
		;
	alt_write_word((h2p_adcspi_addr + SPI_TXDATA_offst), comm);
	while (!(alt_read_word(h2p_adcspi_addr + SPI_STATUS_offst) & (1 << status_TMT_bit)))
		;
	data = alt_read_word(h2p_adcspi_addr + SPI_RXDATA_offst);
	// wait for the spi command to finish
	return (data);
}

void write_beamformer_spi(unsigned char spi_reg_length, unsigned char read, unsigned char spi_addr, unsigned long spi_data_out, unsigned int *spi_in0, unsigned int *spi_in1, unsigned int *spi_in2) {
	unsigned int spi_out0, spi_out1, spi_out2;
	spi_out0 = 0;
	spi_out1 = 0;
	spi_out2 = 0;
	spi_out0 = (spi_addr & 0x1F) | ((read & 0x01) << 5) | ((spi_data_out & 0x3FFFFFF) << 6);
	spi_out1 = (spi_data_out >> 26) & 0xFFFFFFFF;
	// spi_out2 = (unsigned int)(spi_data_out>>58) & (unsigned int) 0x3F; WARNING !

	alt_write_word(h2p_lm96570_spi_in0_addr, spi_out0);
	alt_write_word(h2p_lm96570_spi_in1_addr, spi_out1);
	alt_write_word(h2p_lm96570_spi_in2_addr, spi_out2);
	alt_write_word(h2p_spi_num_of_bits_addr, spi_reg_length + 6);
	// 6 is the command length

	cnt_out_val |= lm96570_start_MSK;
	alt_write_word(h2p_general_cnt_out_addr, cnt_out_val);
	// start the beamformer SPI
	usleep(100);

	cnt_out_val &= (~lm96570_start_MSK);
	alt_write_word(h2p_general_cnt_out_addr, cnt_out_val);
	// stop the beamformer SPI
	*spi_in0 = alt_read_word(h2p_lm96570_spi_out0_addr);
	*spi_in1 = alt_read_word(h2p_lm96570_spi_out1_addr);
	*spi_in2 = alt_read_word(h2p_lm96570_spi_out2_addr);
	//printf("beamformer_spi_in = 0x%04x_%04x_%04x\n", *spi_in2, *spi_in1, *spi_in0);

}

void read_adc_id() {
	unsigned int command, data;
	unsigned int comm, addr;
	comm = (1 << 7) | (0 << 6) | (0 << 5);   // read chip settings
	addr = 0x00;
	command = (comm << 16) | (addr << 8) | 0x00;
	data = write_adc_spi(command);
	//printf("command = 0x%06x -> datain = 0x%06x\n", command, data);
	comm = (1 << 7) | (0 << 6) | (0 << 5);   // read chip ID
	addr = 0x01;
	command = (comm << 16) | (addr << 8) | 0x00;
	data = write_adc_spi(command);
	//printf("command = 0x%06x -> datain = 0x%06x\n", command, data);
	comm = (1 << 7) | (0 << 6) | (0 << 5);   // read chip ID
	addr = 0x02;
	command = (comm << 16) | (addr << 8) | 0x00;
	data = write_adc_spi(command);
	//printf("command = 0x%06x -> datain = 0x%06x\n", command, data);
}
void init_adc() {
	unsigned int command, data;

	unsigned int comm, addr;
	comm = (AD9276_SPI_WR << 7) | AD9276_1BYTE_DATA;
	//addr = 0x04;
	//command = (comm<<16) | (addr<<8) | 0x0F; // select data channel E-H
	//data = write_adc_spi (command);
	//printf("command = 0x%06x -> datain = 0x%06x\n", command, data);
	//addr = 0x05;
	//command = (comm<<16) | (addr<<8) | 0x0F; // select data channel A-D
	//data = write_adc_spi (command);
	//printf("command = 0x%06x -> datain = 0x%06x\n", command, data);
	addr = 0x11;
	command = (comm << 16) | (addr << 8) | (0x0E);   // set PGA Gain to 21 dB, LNA Gain to 15.6 dB
	data = write_adc_spi(command);
	//printf("command = 0x%06x -> datain = 0x%06x\n", command, data);
	addr = 0xFF;
	command = (comm << 16) | (addr << 8) | (0x01);   // update the device
	data = write_adc_spi(command);
	//printf("command = 0x%06x -> datain = 0x%06x\n", command, data);
	addr = 0x15;
	command = (comm << 16) | (addr << 8) | (0x02 << 4);   // set output driver
	// to 100 ohms
	data = write_adc_spi(command);
	//printf("command = 0x%06x -> datain = 0x%06x\n", command, data);
	addr = 0xFF;
	command = (comm << 16) | (addr << 8) | (0x01);   // update the device
	data = write_adc_spi(command);
	//printf("command = 0x%06x -> datain = 0x%06x\n", command, data);
	addr = 0x16;
	command = (comm << 16) | (addr << 8) | (0x08);   // set output phase
	// to 480 degrees
	data = write_adc_spi(command);
	//printf("command = 0x%06x -> datain = 0x%06x\n", command, data);
	addr = 0xFF;

	command = (comm << 16) | (addr << 8) | (0x01);   // update the device
	data = write_adc_spi(command);
	// printf("command = 0x%06x -> datain = 0x%06x\n", command, data);
	// read register
	comm = (AD9276_SPI_WR << 7) | AD9276_1BYTE_DATA;
	addr = 0x05;
	command = (comm << 16) | (addr << 8) | 0x01;   // select data channel A
	data = write_adc_spi(command);
	//printf("command = 0x%06x -> datain = 0x%06x\n", command, data);
	comm = (AD9276_SPI_RD << 7) | AD9276_1BYTE_DATA;
	addr = 0x15;
	command = (comm << 16) | (addr << 8) | 0x00;   // read channel A
	// termination status
	data = write_adc_spi(command);
	//printf("command = 0x%06x -> datain = 0x%06x\n", command, data);
	addr = 0x05;
	command = (comm << 16) | (addr << 8) | 0x01;   // select data channel A
	data = write_adc_spi(command);
	//printf("command = 0x%06x -> datain = 0x%06x\n", command, data);
	comm = (AD9276_SPI_RD << 7) | AD9276_1BYTE_DATA;
	addr = 0x11;
	command = (comm << 16) | (addr << 8) | 0x00;   // read channel A Gain status
	data = write_adc_spi(command);
	//printf("command = 0x%06x -> datain = 0x%06x\n", command, data);
	addr = 0x05;
	command = (comm << 16) | (addr << 8) | 0x01;   // select data channel A
	data = write_adc_spi(command);
	//printf("command = 0x%06x -> datain = 0x%06x\n", command, data);
	comm = (AD9276_SPI_RD << 7) | AD9276_1BYTE_DATA;
	addr = 0x16;
	command = (comm << 16) | (addr << 8) | 0x00;   // read channel A Phase status
	data = write_adc_spi(command);
	//printf("command = 0x%06x -> datain = 0x%06x\n", command, data);

}
void init_beamformer() {
	unsigned int spi_in0, spi_in1, spi_in2;
	spi_in0 = 0x00;
	spi_in1 = 0x00;
	spi_in2 = 0x00;
	write_beamformer_spi(REG_1A_LENGTH, LM86570_SPI_WR, 0x1A, 0x4040, &spi_in0, &spi_in1, &spi_in2);   //Default: 0x4600, Last Zero needed for padding.
	write_beamformer_spi(REG_00_07_LENGTH, LM86570_SPI_WR, 0x00, 0x0000, &spi_in0, &spi_in1, &spi_in2);   //D0:0x0000, D1:0x0000, D2:0x0000, D3:0x0000, D4:0x02C7, D5:0x057A
	write_beamformer_spi(REG_00_07_LENGTH, LM86570_SPI_WR, 0x01, 0x0000, &spi_in0, &spi_in1, &spi_in2);   //D0:0x0000, D1:0x0103, D2:0x00A4, D3:0x003F, D4:0x02A0, D5:0x04EE
	write_beamformer_spi(REG_00_07_LENGTH, LM86570_SPI_WR, 0x02, 0x0000, &spi_in0, &spi_in1, &spi_in2);   //D0:0x0000, D1:0x01F4, D2:0x0133, D3:0x006A, D4:0x0265, D5:0x044E
	write_beamformer_spi(REG_00_07_LENGTH, LM86570_SPI_WR, 0x03, 0x0000, &spi_in0, &spi_in1, &spi_in2);   //D0:0x0000, D1:0x02D1, D2:0x01AE, D3:0x007F, D4:0x0214, D5:0x0399
	write_beamformer_spi(REG_00_07_LENGTH, LM86570_SPI_WR, 0x04, 0x0000, &spi_in0, &spi_in1, &spi_in2);   //D0:0x0000, D1:0x0399, D2:0x0214, D3:0x007F, D4:0x01AE, D5:0x02D1
	write_beamformer_spi(REG_00_07_LENGTH, LM86570_SPI_WR, 0x05, 0x0000, &spi_in0, &spi_in1, &spi_in2);   //D0:0x0000, D1:0x044E, D2:0x0265, D3:0x006A, D4:0x0133, D5:0x01F4
	write_beamformer_spi(REG_00_07_LENGTH, LM86570_SPI_WR, 0x06, 0x0000, &spi_in0, &spi_in1, &spi_in2);   //D0:0x0000, D1:0x04EE, D2:0x02A0, D3:0x003F, D4:0x00A4, D5:0x0103
	write_beamformer_spi(REG_00_07_LENGTH, LM86570_SPI_WR, 0x07, 0x0000, &spi_in0, &spi_in1, &spi_in2);   //D0:0x0000, D1:0x057A, D2:0x02C7, D3:0x0000, D4:0x0000, D5:0x0000
	//write_beamformer_spi(4, LM86570_SPI_WR, 0x08, 0x0005, &spi_in0, &spi_in1, &spi_in2); //Default: 0x0005
	//write_beamformer_spi(4, LM86570_SPI_WR, 0x09, 0x0005, &spi_in0, &spi_in1, &spi_in2); //Default: 0x0005

	//write_beamformer_spi(4, LM86570_SPI_WR, 0x0A, 0x0005, &spi_in0, &spi_in1, &spi_in2); //Default: 0x0005
	//write_beamformer_spi(4, LM86570_SPI_WR, 0x0B, 0x0005, &spi_in0, &spi_in1, &spi_in2); //Default: 0x0005
	//write_beamformer_spi(4, LM86570_SPI_WR, 0x0C, 0x0005,&spi_in0, &spi_in1, &spi_in2); //Default: 0x0005
	//write_beamformer_spi(4, LM86570_SPI_WR, 0x0D, 0x0005,&spi_in0, &spi_in1, &spi_in2); //Default: 0x0005
	//write_beamformer_spi(4, LM86570_SPI_WR, 0x0E, 0x0005,&spi_in0, &spi_in1, &spi_in2); //Default: 0x0005
	//write_beamformer_spi(4, LM86570_SPI_WR, 0x0F, 0x0005,&spi_in0, &spi_in1, &spi_in2); //Default: 0x0005
	//write_beamformer_spi(4, LM86570_SPI_WR, 0x10, 0x000A,&spi_in0, &spi_in1, &spi_in2); //Default: 0x000A
	//write_beamformer_spi(4, LM86570_SPI_WR, 0x11, 0x000A,&spi_in0, &spi_in1, &spi_in2); //Default: 0x000A
	//write_beamformer_spi(4, LM86570_SPI_WR, 0x12, 0x000A,&spi_in0, &spi_in1, &spi_in2); //Default: 0x000A
	//write_beamformer_spi(4, LM86570_SPI_WR, 0x13, 0x000A,&spi_in0, &spi_in1, &spi_in2); //Default: 0x000A
	//write_beamformer_spi(4, LM86570_SPI_WR, 0x14, 0x000A,&spi_in0, &spi_in1, &spi_in2); //Default: 0x000A
	//write_beamformer_spi(4, LM86570_SPI_WR, 0x15, 0x000A,&spi_in0, &spi_in1, &spi_in2); //Default: 0x000A
	//write_beamformer_spi(4, LM86570_SPI_WR, 0x16, 0x000A,&spi_in0, &spi_in1, &spi_in2); //Default: 0x000A
	//write_beamformer_spi(4, LM86570_SPI_WR, 0x17, 0x000A,&spi_in0, &spi_in1, &spi_in2); //Default: 0x000A
	write_beamformer_spi(4, LM86570_SPI_WR, 0x18, 0x0005, &spi_in0, &spi_in1, &spi_in2);   // write all p registers, Default: 0x0005, 4
	write_beamformer_spi(4, LM86570_SPI_WR, 0x19, 0x000A, &spi_in0, &spi_in1, &spi_in2);   // write all n registers, Default: 0x000A, 4
	write_beamformer_spi(4, REG_1BA_LENGTH, 0x1B, 0x0000, &spi_in0, &spi_in1, &spi_in2);   //Default: 0x0000
}

void read_adc_val(void *channel_csr_addr, void *channel_data_addr, unsigned int * adc_data) {   //, char *filename) {
	unsigned int fifo_mem_level;
	//fptr = fopen(filename, "w");
	//if (fptr == NULL) {
	// printf("File does not exists \n");
	// return;
	//}
	// PRINT # of DATAS in FIFO
	fifo_mem_level = alt_read_word(channel_csr_addr + ALTERA_AVALON_FIFO_LEVEL_REG);
	// the fill level of FIFO memory
	//printf("num of data in fifo: %d\n",fifo_mem_level);
	//
	// READING DATA FROM FIFO
	fifo_mem_level = alt_read_word(channel_csr_addr + ALTERA_AVALON_FIFO_LEVEL_REG);
	// the fill level of FIFO memory
	for (i = 0; fifo_mem_level > 0; i++) {
		adc_data[i] = alt_read_word(channel_data_addr);
		//fprintf(fptr, "%d\n", rddata[i] & 0xFFF);
		//fprintf(fptr, "%d\n", (rddata[i]>>16) & 0xFFF);
		fifo_mem_level--;
		if (fifo_mem_level == 0) {
			fifo_mem_level = alt_read_word(channel_csr_addr + ALTERA_AVALON_FIFO_LEVEL_REG);
		}
	}
	usleep(10);
	fifo_mem_level = alt_read_word(channel_csr_addr + ALTERA_AVALON_FIFO_LEVEL_REG);
	// the fill level of FIFO memory
	//printf("num of data in fifo: %d\n",fifo_mem_level);
	//fclose(fptr);
}

void store_data(unsigned int * adc_data, unsigned int data_bank[num_of_switches][num_of_channels][num_of_samples], unsigned int sw_num, unsigned int ch_num, unsigned int num_of_samples) {
	unsigned int ii;
	for (ii = 0; ii < num_of_samples / 2; ii++) {
		data_bank[sw_num][ch_num][ii * 2] = adc_data[ii] & 0xFFF;
		data_bank[sw_num][ch_num][ii * 2 + 1] = (adc_data[ii] >> 16) & 0xFFF;
	}
}
void write_data_bank(unsigned int data_bank[num_of_switches][num_of_channels][num_of_samples]) {
	unsigned int ii, jj, kk;
	fptr = fopen("databank.txt", "w");
	if (fptr == NULL) {
		printf("File does not exists \n");
		return;
	}
	for (ii = 0; ii < num_of_switches; ii++) {
		for (jj = 0; jj < num_of_channels; jj++) {
			for (kk = 0; kk < num_of_samples; kk++) {
				fprintf(fptr, "%d ", data_bank[ii][jj][kk] & 0xFFF);
			}
			fprintf(fptr, "\n");
		}
		//fprintf(fptr, "\n");
	}
}
void print_data_bank(unsigned int data_bank[num_of_switches][num_of_channels][num_of_samples]) {
	unsigned int ii, jj, kk;
	/*fptr = fopen("databank.txt", "w");
	 if (fptr == NULL) {
	 printf("File does not exists \n");
	 return;
	 }*/
	for (ii = 0; ii < num_of_switches; ii++) {
		for (jj = 0; jj < num_of_channels; jj++) {

			for (kk = 0; kk < num_of_samples; kk++) {
				printf("%d ", data_bank[ii][jj][kk] & 0xFFF);
			}
			//fprintf(fptr, "\n");
		}
		//fprintf(fptr, "\n");
	}
}
int main() {
// Initialize system
	init();
	read_adc_id();
	init_beamformer();
	init_adc();
	alt_write_word(h2p_adc_start_pulselength_addr, 10);
	alt_write_word(h2p_adc_samples_per_echo_addr, num_of_samples);
	unsigned int data_bank[num_of_switches][num_of_channels][num_of_samples];
	unsigned int adc_data[num_of_samples];   // data for 1 acquisition
	unsigned int sw_num = 0;
	for (sw_num = 0; sw_num < num_of_switches; sw_num++) {
		//Reset FSM in order to address glitching
		cnt_out_val |= FSM_RST_MSK;
		alt_write_word(h2p_general_cnt_out_addr, cnt_out_val);
		// restart the Ultrasound FSM
		usleep(1000);
		cnt_out_val &= (~FSM_RST_MSK);
		alt_write_word(h2p_general_cnt_out_addr, cnt_out_val);
		// set the Ultrasound FSM
		usleep(1000);
		//write mux
		alt_write_word(h2p_mux_control_addr, ((0x00) & 0x7FF));
		usleep(500);
		alt_write_word(h2p_mux_control_addr, ((1 << sw_num) & 0x7FF));

		init_beamformer();
		usleep(500);
		// tx_path on
		//cnt_out_val |= tx_path_en_MSK;
		//alt_write_word( h2p_general_cnt_out_addr , cnt_out_val);
		// start the beamformer SPI
		//usleep(100000);
		// sw_off on
		//cnt_out_val |= sw_off_MSK;
		//alt_write_word( h2p_general_cnt_out_addr , cnt_out_val);
		// start the beamformer SPI
		//usleep(100000);
		// pulser on
		cnt_out_val |= pulser_en_MSK;
		alt_write_word(h2p_general_cnt_out_addr, cnt_out_val);
		// start the beamformer SPI
		usleep(1000);
		// tx_enable fire
		cnt_out_val |= lm96570_tx_en_MSK;
		alt_write_word(h2p_general_cnt_out_addr, cnt_out_val);
		// start the beamformer SPI
		usleep(1000);
		cnt_out_val &= (~lm96570_tx_en_MSK);
		alt_write_word(h2p_general_cnt_out_addr, cnt_out_val);
		// stop the beamformer SPI
		// pulser off
		//usleep(200000);
		cnt_out_val &= (~pulser_en_MSK);
		alt_write_word(h2p_general_cnt_out_addr, cnt_out_val);
		// stop the beamformer SPI
		read_adc_val(h2p_fifo_sink_ch_a_csr_addr, h2p_fifo_sink_ch_a_data_addr, adc_data);
		store_data(adc_data, data_bank, sw_num, 0, num_of_samples);
		read_adc_val(h2p_fifo_sink_ch_b_csr_addr, h2p_fifo_sink_ch_b_data_addr, adc_data);
		store_data(adc_data, data_bank, sw_num, 1, num_of_samples);
		read_adc_val(h2p_fifo_sink_ch_c_csr_addr, h2p_fifo_sink_ch_c_data_addr, adc_data);
		store_data(adc_data, data_bank, sw_num, 2, num_of_samples);
		read_adc_val(h2p_fifo_sink_ch_d_csr_addr, h2p_fifo_sink_ch_d_data_addr, adc_data);
		store_data(adc_data, data_bank, sw_num, 3, num_of_samples);
		read_adc_val(h2p_fifo_sink_ch_e_csr_addr, h2p_fifo_sink_ch_e_data_addr, adc_data);
		store_data(adc_data, data_bank, sw_num, 4, num_of_samples);
		read_adc_val(h2p_fifo_sink_ch_f_csr_addr, h2p_fifo_sink_ch_f_data_addr, adc_data);
		store_data(adc_data, data_bank, sw_num, 5, num_of_samples);
		read_adc_val(h2p_fifo_sink_ch_g_csr_addr, h2p_fifo_sink_ch_g_data_addr, adc_data);
		store_data(adc_data, data_bank, sw_num, 6, num_of_samples);
		read_adc_val(h2p_fifo_sink_ch_h_csr_addr, h2p_fifo_sink_ch_h_data_addr, adc_data);
		store_data(adc_data, data_bank, sw_num, 7, num_of_samples);
		//printf("Completed Event: %d\n",sw_num);
	}
	write_data_bank(data_bank);
	print_data_bank(data_bank);
	// exit program
	leave();
	//printf("%s \n",data_bank);
	return 0;
}
