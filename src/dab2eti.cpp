/*
This file is part of rtl-dab
trl-dab is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

Foobar is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with rtl-dab.  If not, see <http://www.gnu.org/licenses/>.


david may 2012
david.may.muc@googlemail.com

*/


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <semaphore.h>
#include <signal.h>
#include <rtl-sdr.h>
#include <libhackrf/hackrf.h>
#include <unistd.h>
#include <string>
#include <cmath>
#include "dab.h"
extern "C" {
#include "input_sdr.h"
#include "input_wf.h"
}

/* Wavefinder state */
static struct wavefinder_t  wf;

/* RTL-SDR device state */
static struct sdr_state_t sdr;
static rtlsdr_dev_t *dev = NULL;

/* HackRF device state */
static hackrf_device *hackrf = NULL;
static uint32_t num_saturated = 0; // Number of saturated samples in the last buffer
static uint32_t num_low_power = 0; // Number of samples in the last buffer with low power
static unsigned int hackrf_lna_gain=8; // 8dB steps
const unsigned int hackrf_lna_gain_min = 0;
const unsigned int hackrf_lna_gain_max = 40;

static unsigned int hackrf_vga_gain=2; // 2dB steps
const unsigned int hackrf_vga_gain_min = 0;
const unsigned int hackrf_vga_gain_max = 62;

int do_exit = 0;

static pthread_t demod_thread;
static sem_t data_ready;

#define AUTO_GAIN -100
#define DEFAULT_ASYNC_BUF_NUMBER 32

uint32_t corr_counter;
uint32_t ccount=0;

static void sighandler(int signum)
{
  fprintf(stderr, "Signal caught, exiting!\n");
  do_exit = 1;
  rtlsdr_cancel_async(dev);
}

static void hackrf_err(const char* msg, int result) {
  fprintf(stderr, "%s: %s (%d)\n", msg, hackrf_error_name((enum hackrf_error)result), result);
}

static void set_center_freq(struct dab_state_t *dab, long frequency)
{
  if (dab->device_type == DAB_DEVICE_RTLSDR) {
    rtlsdr_set_center_freq(dev, frequency);
  }
  else if (dab->device_type == DAB_DEVICE_HACKRF) {
    hackrf_set_freq(hackrf, frequency);
  }
  else {
    abort();
  }
}

static void *demod_thread_fn(void *ptr)
{
  struct dab_state_t *dab = (struct dab_state_t*)ptr;
  struct sdr_state_t *sdr = (struct sdr_state_t*)dab->device_state;
  int i,j;

  while (!do_exit) {
    sem_wait(&data_ready);
    int ok = sdr_demod(&dab->tfs[dab->tfidx], sdr);
    if (ok) {
      dab_process_frame(dab);
    }
    //dab_fic_parser(dab->fib,&sinfo,&ana);
    // calculate error rates
    //dab_analyzer_calculate_error_rates(&ana,dab);

    int prev_freq = sdr->frequency;
    if (abs(sdr->coarse_freq_shift)>1) {
      if (sdr->coarse_freq_shift<0)
        sdr->frequency = sdr->frequency -1000;
      else
        sdr->frequency = sdr->frequency +1000;

      set_center_freq(dab, sdr->frequency);
    }

    if (abs(sdr->coarse_freq_shift) ==1) {

      if (sdr->coarse_freq_shift<0)
        sdr->frequency = sdr->frequency -rand() % 1000;
      else
        sdr->frequency = sdr->frequency +rand() % 1000;

      set_center_freq(dab, sdr->frequency);
      //fprintf(stderr,"new center freq : %i\n",rtlsdr_get_center_freq(dev));

    }
    if (abs(sdr->coarse_freq_shift)<1 && (fabs(sdr->fine_freq_shift) > 50)) {
      sdr->frequency = sdr->frequency + (sdr->fine_freq_shift/3);
      set_center_freq(dab, sdr->frequency);
      //fprintf(stderr,"ffs : %f\n",sdr->fine_freq_shift);
    }

    //if (sdr->frequency != prev_freq) {
    //  fprintf(stderr,"Adjusting centre-frequency to %dHz\n",sdr->frequency);
    //}    
    ccount += 1;
    if (ccount == 10) {
      ccount = 0;
      //print_status(dab);
    }
  }
  return 0;
}

static int hackrf_callback(hackrf_transfer* transfer)
{
  if (do_exit) {
    return -1;
  }
  if (!transfer) {
    return -1;
  }

  struct sdr_state_t *sdr = (struct sdr_state_t*)transfer->rx_ctx;

  size_t len = transfer->valid_length;
  const uint8_t* buf = transfer->buffer;

  if (len > DEFAULT_BUF_LENGTH) abort();
  memcpy(sdr->input_buffer,buf,len);
  sdr->input_buffer_len = len;

  num_saturated = 0;
  num_low_power = 0;
  for (size_t i = 0; i < len; i += 1) {

    int8_t val = buf[i];

    if (val > 125 || val < -125) {
      num_saturated ++;
    }
    if (val > -5 && val < 5) {
      num_low_power ++;
    }
  }

  int dr_val;
  sem_getvalue(&data_ready, &dr_val);
  if (!dr_val) {
    sem_post(&data_ready);
  }
  return 0;
}

static void rtlsdr_callback(uint8_t *buf, uint32_t len, void *ctx)
{
  struct sdr_state_t *sdr = (struct sdr_state_t*)ctx;
  int dr_val;
  if (do_exit) {
    return;}
  if (!ctx) {
    return;}
  memcpy(sdr->input_buffer,buf,len);
  sdr->input_buffer_len = len;
  sem_getvalue(&data_ready, &dr_val);
  if (!dr_val) {
    sem_post(&data_ready);}
}

static void eti_callback(uint8_t* eti)
{
  write(1, eti, 6144);
}

static int do_sdr_decode(struct dab_state_t* dab, int frequency, int gain)
{
  struct sigaction sigact;
  uint32_t dev_index = 0;
  int32_t device_count;
  int i,r;
  char vendor[256], product[256], serial[256];
  uint32_t samp_rate = 2048000;

  memset(&sdr,0,sizeof(struct sdr_state_t));

  sdr.frequency = frequency;

  //fprintf(stderr,"%i\n",sdr.frequency);

  /*---------------------------------------------------
    Looking for device and open connection
    ----------------------------------------------------*/
  if (dab->device_type == DAB_DEVICE_RTLSDR) {
    sdr.convert_unsigned = 1;

    device_count = rtlsdr_get_device_count();
    if (!device_count) {
      fprintf(stderr, "No supported devices found.\n");
      exit(1);
    }

    fprintf(stderr, "Found %d device(s):\n", device_count);
    for (i = 0; i < device_count; i++) {
      rtlsdr_get_device_usb_strings(i, vendor, product, serial);
      fprintf(stderr, "  %d:  %s, %s, SN: %s\n", i, vendor, product, serial);
    }
    fprintf(stderr, "\n");

    fprintf(stderr, "Using device %d: %s\n",dev_index, rtlsdr_get_device_name(dev_index));

    r = rtlsdr_open(&dev, dev_index);
    if (r < 0) {
      fprintf(stderr, "Failed to open rtlsdr device #%d.\n", dev_index);
      exit(1);
    }

    int gains[100];
    int count = rtlsdr_get_tuner_gains(dev, gains);
    fprintf(stderr, "Supported gain values (%d): ", count);
    for (i = 0; i < count; i++)
      fprintf(stderr, "%.1f ", gains[i] / 10.0);
    fprintf(stderr, "\n");
  }
  else if (dab->device_type == DAB_DEVICE_HACKRF) {
    sdr.convert_unsigned = 0;
    r = hackrf_init();
    if( r != HACKRF_SUCCESS ) {
      hackrf_err("hackrf_init() failed", r);
      return EXIT_FAILURE;
    }

    const char* serial_number = nullptr;
    r = hackrf_open_by_serial(serial_number, &hackrf);
    if( r != HACKRF_SUCCESS ) {
      hackrf_err("hackrf_open() failed", r);
      return EXIT_FAILURE;
    }
  }
  else
  {
    r = -1;
    return EXIT_FAILURE;
  }

  /*-------------------------------------------------
    Set Frequency & Sample Rate
    --------------------------------------------------*/
  if (dab->device_type == DAB_DEVICE_RTLSDR) {
    /* Set the sample rate */
    r = rtlsdr_set_sample_rate(dev, samp_rate);
    if (r < 0)
      fprintf(stderr, "WARNING: Failed to set sample rate.\n");

    /* Set the frequency */
    r = rtlsdr_set_center_freq(dev, sdr.frequency);
    if (r < 0)
      fprintf(stderr, "WARNING: Failed to set center freq.\n");
    else
      fprintf(stderr, "Tuned to %u Hz.\n", sdr.frequency);

    /*------------------------------------------------
      Setting gain  
      -------------------------------------------------*/
    if (gain == AUTO_GAIN) {
      r = rtlsdr_set_tuner_gain_mode(dev, 0);
    } else {
      r = rtlsdr_set_tuner_gain_mode(dev, 1);
      r = rtlsdr_set_tuner_gain(dev, gain);
    }
    if (r != 0) {
      fprintf(stderr, "WARNING: Failed to set tuner gain.\n");
    } else if (gain == AUTO_GAIN) {
      fprintf(stderr, "Tuner gain set to automatic.\n");
    } else {
      fprintf(stderr, "Tuner gain set to %0.2f dB.\n", gain/10.0);
    }
    /*-----------------------------------------------
      /  Reset endpoint (mandatory) 
      ------------------------------------------------*/
    r = rtlsdr_reset_buffer(dev);
  }
  else if (dab->device_type == DAB_DEVICE_HACKRF) {
    int sample_rate_hz = samp_rate;
    fprintf(stderr, "call hackrf_sample_rate_set(%u Hz/%.03f MHz)\n", sample_rate_hz, (sample_rate_hz/1e6));
    int r = hackrf_set_sample_rate_manual(hackrf, sample_rate_hz, 1);
    if( r != HACKRF_SUCCESS ) {
      hackrf_err("hackrf_sample_rate_set() failed", r);
      return EXIT_FAILURE;
    }

    /* possible settings 1.75/2.5/3.5/5/5.5/6/7/8/9/10/12/14/15/20/24/28 */
    int baseband_filter_bw_hz = 2500000;
    fprintf(stderr, "call hackrf_baseband_filter_bandwidth_set(%d Hz/%.03f MHz)\n",
        baseband_filter_bw_hz, ((float)baseband_filter_bw_hz/1e6));
    r = hackrf_set_baseband_filter_bandwidth(hackrf, baseband_filter_bw_hz);
    if( r != HACKRF_SUCCESS ) {
      hackrf_err("hackrf_baseband_filter_bandwidth_set()", r);
      return EXIT_FAILURE;
    }

    r = hackrf_set_vga_gain(hackrf, hackrf_vga_gain);
    r |= hackrf_set_lna_gain(hackrf, hackrf_lna_gain);

    if( r != HACKRF_SUCCESS ) {
      hackrf_err("hackrf_vga gain/lna gain", r);
      return EXIT_FAILURE;
    }

    r = hackrf_set_freq(hackrf, sdr.frequency);
    if( r != HACKRF_SUCCESS ) {
      hackrf_err("hackrf_set_freq()", r);
      return EXIT_FAILURE;
    }
  }
  /*-----------------------------------------------
  / Signal handler
  ------------------------------------------------*/
  sigact.sa_handler = sighandler;
  sigemptyset(&sigact.sa_mask);
  sigact.sa_flags = 0;
  sigaction(SIGINT, &sigact, NULL);
  sigaction(SIGTERM, &sigact, NULL);
  sigaction(SIGQUIT, &sigact, NULL);
  sigaction(SIGPIPE, &sigact, NULL);
  /*-----------------------------------------------
  / start demod thread & rtl read 
  -----------------------------------------------*/

  fprintf(stderr,"Waiting for sync...\n");

  sdr_init(&sdr);
  //dab_fic_parser_init(&sinfo);
  //dab_analyzer_init(&ana);
  pthread_create(&demod_thread, NULL, demod_thread_fn, (void *)(dab));
  if (dab->device_type == DAB_DEVICE_RTLSDR) {
    rtlsdr_read_async(dev, rtlsdr_callback, (void *)(&sdr),
        DEFAULT_ASYNC_BUF_NUMBER, DEFAULT_BUF_LENGTH);
  }
  else if (dab->device_type == DAB_DEVICE_HACKRF) {
    r = hackrf_start_rx(hackrf, hackrf_callback, (void *)(&sdr));

    if( r != HACKRF_SUCCESS ) {
      hackrf_err("hackrf_start_x()", r);
      return EXIT_FAILURE;
    }

    while( ((r=hackrf_is_streaming(hackrf)) == HACKRF_TRUE) &&
        (do_exit == false) ) {
      sleep(1);
      fprintf(stderr, "samples: low: %02.2f%%, saturating: %02.2f%%\n",
          num_low_power * 100.0 / DEFAULT_BUF_LENGTH,
          num_saturated * 100.0 / DEFAULT_BUF_LENGTH);
    }
    hackrf_err("hackrf_is_streaming", r);
  }


  if (do_exit) {
    fprintf(stderr, "\nUser cancel, exiting...\n");}
  else {
    fprintf(stderr, "\nLibrary error %d, exiting...\n", r);}
  if (dab->device_type == DAB_DEVICE_RTLSDR) {
    rtlsdr_cancel_async(dev);
    //dab_demod_close(&dab);
    rtlsdr_close(dev);
  }
  else if (dab->device_type == DAB_DEVICE_HACKRF) {
    if (hackrf != NULL)
    {
      r = hackrf_stop_rx(hackrf);
      if( r != HACKRF_SUCCESS ) {
        hackrf_err("hackrf_stop_rx() failed", r);
      }
      else {
        fprintf(stderr, "hackrf_stop_rx() done\n");
      }

      r = hackrf_close(hackrf);
      if( r != HACKRF_SUCCESS )
      {
        hackrf_err("hackrf_close() failed", r);
      }
      else {
        fprintf(stderr, "hackrf_close() done\n");
      }
    }

    hackrf_exit();
  }
  return 1;
}

static int do_wf_decode(struct dab_state_t* dab, int frequency)
{
  struct wavefinder_t *wf = (struct wavefinder_t*)dab->device_state;
  int displayed_lock = 0;

  wf_init(wf);
  wf_tune(wf, (frequency+500)/1000);  /* Round frequency to the nearest KHz */

  fprintf(stderr,"Waiting for sync...");

  /* Read (and discard) the first frame - we know it is missing the FIC symbols */
  wf_read_frame(wf,&dab->tfs[0]);
  if ((wf->sync_locked) && (!displayed_lock)) {
    fprintf(stderr,"LOCKED\n");
    displayed_lock = 1;
  }

  while (1) {
    wf_read_frame(wf,&dab->tfs[dab->tfidx]);
    dab_process_frame(dab);
  }
}

void usage(void)
{
  fprintf(stderr,"Usage: dab2eti (-r|-h) frequency [gain]\n");
  fprintf(stderr,"       -w selects wavefinder\n");
  fprintf(stderr,"       -r selects rtlsdr\n");
  fprintf(stderr,"       -h selects hackrf, gain is [vga gain] [lna gain]\n");
}

int main(int argc, char* argv[])
{
  int frequency;
  int gain = AUTO_GAIN;
  struct dab_state_t* dab;

  if ((argc < 3) || (argc > 5)) {
    usage();
    return 1;
  }

  frequency = atoi(argv[2]);

  std::string device_arg(argv[1]);

  if (device_arg == "-w") {
    if (wf_open(&wf,"/dev/wavefinder0") >= 0) {
      init_dab_state(&dab,&wf,eti_callback);
      dab->device_type = DAB_DEVICE_WAVEFINDER;
      do_wf_decode(dab,frequency);
    }
    else {
      fprintf(stderr, "Could not open wavefinder\n");
    }
  }
  else if (device_arg == "-r") {
    if (argc > 3) { gain = atoi(argv[3]); }
    init_dab_state(&dab,&sdr,eti_callback);
    dab->device_type = DAB_DEVICE_RTLSDR;
    do_sdr_decode(dab,frequency,gain);
  }
  else if (device_arg == "-h") {
    if (argc > 3) { hackrf_vga_gain = atoi(argv[3]); }
    if (argc > 4) { hackrf_lna_gain = atoi(argv[4]); }
    init_dab_state(&dab,&sdr,eti_callback);
    dab->device_type = DAB_DEVICE_HACKRF;
    do_sdr_decode(dab,frequency,gain);
  }
  else {
    usage();
    return 1;
  }
}
