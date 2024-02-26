/* Alesis MultiMix 8 USB 2.0 interface to Jack Audio Connection Kit
 * (c) Stuart Ashby, 2024
 *
 * This is a userspace jack client to interface to the Alesis mixer and exposes:
 * 10 separate inputs (8 channels, 2 mix bus)
 * 2 separate outputs (2 mix bus)
 * This ONLY WORKS at 96kHz (why would you use less?) so setup jackd to this sample rate before use
 *
 * CAVEATS:
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include <stdio.h>
#include <stdarg.h>
#include <unistd.h> // sleep()
#include <stdlib.h> // malloc()/free()
#include <sys/time.h>   // timeval
#include <sys/ioctl.h>	// key handler
#include <string.h>
#include <math.h> // round

#include <jack/jack.h>
#include <jack/ringbuffer.h>

#include "libusb-1.0/libusb.h"

#define RB_FRAME_LENGTH		3072
#define RB_TARGET_LENGTH	768
#define IB_FRAME_LENGTH		8192
#define IB_TARGET_LENGTH	1536

#define AVGSCALE		300	// scale factor used to update ring buffer moving avergae per jack period. divisor
#define DEADBAND		48	// how many frames off target before we make a resample adjustment? 96 frames = 1ms

// Alesis MultiMix8 USB 2.0, 24-bit 96kHz stereo out, 10 channels in (private syntax)
#define targetVendorId			0x13b2
#define targetProductId			0x0030
#define targetOutput			{0,1,2}	// interface, altsetting, endpointaddr, ...
#define targetInput			{1,1,0x81,0x86} // ISO, BULK eps
#define control1			{0x22,1,0x0100,0x0086,3}	// rqType, rqst, wVal, windx, wlength
#define data1				{0x00,0x77,0x01} // uchar - 96000 in LE 24bit
#define control2			{0x22,1,0x0100,0x0002,3}	// rqType, rqst, wVal, windx, wlength
#define data2				{0x00,0x77,0x01} // uchar
#define control3			{0x40,0x49,0x0030,0x0000,0}	// rqType, rqst, wVal, windx, wlength
#define ctlRepeat			1
#define preload				7
#define outpreload			3
#define fbAdjust			3
#define innames				{"ch1","ch3","ch5","ch7","mixL","ch2","ch4","ch6","ch8","mixR"}
#define outnames			{"2trackL","2trackR"}

#define PLAY_LATENCY		(480*outpreload+RB_TARGET_LENGTH) // frames, 480 * USB preload queue plus the ring buffer
#define CAP_LATENCY		(2048+IB_TARGET_LENGTH) // frames, 2048 in USB BULK transfer plus the ring buffer

// hacked from libmaru
#define USB_CLASS_AUDIO                1
#define USB_SUBCLASS_AUDIO_CONTROL     1
#define USB_SUBCLASS_AUDIO_STREAMING   2

#define USB_ENDPOINT_ISOCHRONOUS       0x01
#define USB_ENDPOINT_ASYNC             0x04
#define USB_ENDPOINT_ADAPTIVE          0x08

#define USB_CLASS_DESCRIPTOR           0x20
#define USB_INTERFACE_DESCRIPTOR_TYPE  0x04
#define USB_GENERAL_DESCRIPTOR_SUBTYPE 0x01
#define USB_FORMAT_DESCRIPTOR_SUBTYPE  0x02
#define USB_FORMAT_TYPE_I              0x01

#define USB_AUDIO_FEEDBACK_SIZE        3
#define USB_MAX_CONTROL_SIZE           64

#define USB_REQUEST_UAC_SET_CUR        0x01
#define USB_REQUEST_UAC_GET_CUR        0x81
#define USB_REQUEST_UAC_GET_MIN        0x82
#define USB_REQUEST_UAC_GET_MAX        0x83
#define USB_UAC_VOLUME_SELECTOR        0x02
#define UAS_FREQ_CONTROL               0x01
#define UAS_PITCH_CONTROL              0x02
#define USB_REQUEST_DIR_MASK           0x80
#define UAC_TYPE_SPEAKER               0x0301
#define UAC_OUTPUT_TERMINAL            0x03
#define UAC_FEATURE_UNIT               0x06
struct usb_uas_format_descriptor
{
   uint8_t bLength;
   uint8_t bDescriptorType;
   uint8_t bDescriptorSubtype;
   uint8_t bFormatType;
   uint8_t bNrChannels;
   uint8_t nSubFrameSize;
   uint8_t nBitResolution;
   uint8_t bSamFreqType;
   uint8_t tSamFreq[];
} __attribute__((packed));

// globals
static int done = 0;

static libusb_context *ctx = NULL;

static int outDelta = 0; // tracks if we need to add/remove a sample to the next block - value accumulates the delta reported by feedback ISO packets

jack_port_t *output_port[10];
jack_port_t *input_port[2];
jack_client_t *client;
int running = 0;

// some consts to calculate for later
const size_t sample_size = sizeof(jack_default_audio_sample_t);
const size_t ibframe = 10*sample_size;
const size_t ibsize = ibframe*IB_FRAME_LENGTH;
const size_t ibtlow = ibframe*(IB_TARGET_LENGTH-DEADBAND);
const size_t ibthigh = ibframe*(IB_TARGET_LENGTH+DEADBAND);
const size_t rbframe = 2*sample_size;
const size_t rbsize = rbframe*RB_FRAME_LENGTH;
const size_t rbtlow = rbframe*(RB_TARGET_LENGTH-DEADBAND);
const size_t rbthigh = rbframe*(RB_TARGET_LENGTH+DEADBAND);

// ring buffer for 10 channel flow from USB in
jack_ringbuffer_t *ib;
long ibdrop = 0;
long ibadd = 0;
float ibavg = 0;

// ring buffer for 2 channel flow to USB out
jack_ringbuffer_t *rb;
long rbdrop = 0;
long rbadd = 0;
long rbavg = 0;

// Logging function - treat as printf(...) with leading level
// lvl: debug=0
int debug=0;
void logger(int lvl, const char *fmt, ...) {
	if(lvl==0 && debug==0) return;
	time_t timer;
	char now[26];
	struct tm* tm_info;

	timer = time(NULL);
	tm_info = localtime(&timer);

	strftime(now, 26, "%Y-%m-%d %H:%M:%S", tm_info);
	fprintf(stderr,"[%s] ",now);
	va_list argptr;
	va_start(argptr, fmt);
	vfprintf(stderr, fmt, argptr);
	va_end(argptr);
}

// SIGNAL handlers

static void sig_handler(int sig) {
	logger(1,"\nSTOP\n");
	done=1;
}

void jack_shutdown (void *arg)
{
	logger(1,"\nJACK SHUTDOWN!\n");
	done=1;
}

void jack_latency (jack_latency_callback_mode_t mode, void *arg) {
	logger(0,"\nJACK latency callback. Mode=%d\n", mode);
	jack_latency_range_t range;
	if (mode == JackCaptureLatency) {
		for(int i=0; i<10; i++) {
			//jack_port_get_latency_range (input_port[i], mode, &range); // I dont understand this at all?? what is it doing here?
			range.min = CAP_LATENCY;
			range.max = CAP_LATENCY;
			jack_port_set_latency_range (output_port[i], mode, &range);
		}
	} else {
		for(int i=0; i<2; i++) {
			//jack_port_get_latency_range (input_port[i], mode, &range); // I dont understand this at all?? what is it doing here?
			range.min = PLAY_LATENCY;
			range.max = PLAY_LATENCY;
			jack_port_set_latency_range (input_port[i], mode, &range);
		}
	}
}

/**
 * The process callback for this JACK application
 */

int jack_process (jack_nframes_t nframes, void *arg)
{
	jack_default_audio_sample_t *out[10], *in[2];
	
	if(running==0) return 0; // don't process until we are told it's OK.
	
	if(nframes>1024) { logger(1,"JACK: too many frames!%d\n",nframes); return 0; }

	// get the buffers
	//fprintf(stderr,"b");
	for(int i=0; i<10; i++) {
		out[i] = (jack_default_audio_sample_t*)jack_port_get_buffer(output_port[i], nframes);
	}
	for(int i=0; i<2; i++) {
		in[i] = (jack_default_audio_sample_t*)jack_port_get_buffer(input_port[i], nframes);
	}

	// fill output ports from input ring buffer
	int nb = jack_ringbuffer_read_space(ib); // bytes available
	int nr = nframes*ibframe; // bytes needed by jack
	int na = 0; // bytes to transfer
	// check for buffer underrun
	if(nb<nr) {
		logger(1,"\nIN underrun! buf=%d\n",nb);
		// drop the frame to let input catch up
		// reset moving average to depth
		ibavg = nb;
	} else {
		// adjust samples read to keep buffer at target size - clamp to +/- 1 frame per period. Allow for jack internal latency also
		// update moving average of buffer that will be remaining AFTER we read it
		ibavg += ((nb-nr-jack_frames_since_cycle_start(client)*ibframe)/AVGSCALE)-(ibavg/AVGSCALE);
		int sd = 0;
		// clamp to +/- 1 frames
		if(ibavg<ibtlow) { sd = -1; }
		if(ibavg>ibthigh) { sd = 1; }
		na = nr+sd*ibframe; // adjust bytes to read
		na = na>nb ? nb : na; // clamp to available bytes
		ibdrop += sd==1?10:0; // count resample in samples dropped
	}
	static jack_default_audio_sample_t ab[1025*10]; // temp transfer buffer - max 1024 frames! 1 extra allowed for dropping frames
	jack_default_audio_sample_t *pab = ab+(na/sample_size); // pointer to next sample in buffer
	if(na>0) {
		jack_ringbuffer_read(ib, (void *)ab, na); // 
		// duplicate last samples as required
		while(na<nr) {
			*pab = *(pab-10);
			pab++;
			na+=sample_size;
			ibadd++; // count resample in samples added
		}
		pab = ab;
		for(int i=0; i<nframes; i++ ) {
			// fill up outputs from the audio buffer in blocks of 10 channels
			for(int ch=0; ch<10; ch++) {
				*(out[ch]) = *pab;
				out[ch]++;
				pab++;
			}
		}
	}	
	
	// fill output ring buffer from input ports
	pab = ab;
	for(int i=0; i<nframes; i++) {
		for(int ch=0; ch<2; ch++) {
			// interleave into local buffer
			*pab = *in[ch];
			in[ch]++;
			pab++;
		}
	}
	nb = jack_ringbuffer_read_space(rb);
	nr = nframes*rbframe;
	// check for buffer overrun - allow for an extra frame of padding
	if((nr+1)>(na=jack_ringbuffer_write_space(rb))) {
		logger(1,"\nOUT: overrun! space=%d\n",na);
		// drop incoming and reset moving avg to current depth
		rbavg = nb;
	} else {
		// adjust samples written to keep buffer at target size - clamp to +/- 1 frame per period, allow for jack internal latency also
		// update moving average of buffer
		rbavg += ((nb+jack_frames_since_cycle_start(client)*rbframe)/AVGSCALE)-(rbavg/AVGSCALE);
		int sd = 0;
		// clamp to +/- 1 frames
		na = nr;
		if(rbavg<rbtlow) {
			// if too low add a duplicate sample
			*pab = *(pab-2);
			pab++;
			*pab = *(pab-2);
			na += rbframe;
			rbadd++; // count adds
		}
		if(rbavg>rbthigh) {
			// if too high drop a frame
			na -= rbframe;
			rbdrop++; //count drops
		}
		// write to buffer
		if(jack_ringbuffer_write(rb, (void *) ab, na)<na) {
			logger(1,"\nOutput buffer error QUIT\n"); // this should NOT happen!
			return 1;
		}
	}

	return 0;      
}


static void send_control(libusb_device_handle *hdev, uint16_t ctl[], unsigned char * data) {
	int r = 0;
	// blocking API call to send control packet
	logger(0,"control_txfr\n");
	r = libusb_control_transfer(hdev,0xff & ctl[0],0xff & ctl[1],ctl[2],ctl[3],data,ctl[4],0);
	if(r < 0) { logger(1, libusb_strerror(r)); return;}
}

static void cb_out(struct libusb_transfer *transfer)
{
	//fprintf(stderr,"o");
	if(transfer->status != LIBUSB_TRANSFER_COMPLETED) {
		logger(1,"!o\n"); // report failures
	}
	if(transfer->status != LIBUSB_TRANSFER_CANCELLED) {
		//fprintf(stderr,"o");
		// adjust frame size up/down by 1 sample (6 bytes) according to outDelta required.
		// scale factor adjusts sensitivity of feedback loop!
		int sd = outDelta/fbAdjust;
		sd = (sd > 0) - (sd < 0); // reduce to -1/0/+1 range
		if(sd!=0) { 
			outDelta=0; // reset accumulator if we adjusted this transfer
		}
		int nr = rbframe*(480+sd); // bytes required from ring buffer
		transfer->length = 2880+(sd*6); // adjust bytes conveyed in transaction
		transfer->iso_packet_desc[39].length = 72+(sd*6); // adjust size of last ISO subframe to cater!
		// collect audio from ring buffer - pad it out by duplicating if there isn't enough
		int nb = jack_ringbuffer_read_space(rb); // bytes available
		int na = nr; // bytes to actually transfer
		if(nb<nr) {
			logger(1,"\nOUT underrun! buf=%d\n",nb);
			// send zeros, leave samples in buffer
			memset(transfer->buffer,0,transfer->length);
			na = 0;
		}
		static jack_default_audio_sample_t ab[962]; // temp transfer buffer, 480 frames + 1 extra for adjustment
		if(na>0) {
			jack_ringbuffer_read(rb, (void *)ab, na);
			// transcode to S24_3LE into USB output buffer
			jack_default_audio_sample_t *pab = ab;
			char *bp = transfer->buffer;
			for(int i=0; i<480+sd; i++) {
				for(int ch=0; ch<2; ch++) {
					int sample = (*pab)*((float)INT_MAX);
					for(int b=0; b<3; b++) {
						sample >>=8; //shift byte down
						*bp = sample&0xff; // mask and add to output
						bp++; // increment output pointer
					}
					pab++;
				}
			}
		}
		int r=0;
		r = libusb_submit_transfer(transfer); // queue it back up again
		if(r<0) logger(1,"\n%s",libusb_strerror(r));
	}
}


static void fb_in(struct libusb_transfer *transfer)
{
	static int cnt=0;
	//fprintf(stderr,"f");
	if(transfer->status != LIBUSB_TRANSFER_COMPLETED) {
		logger(1,"!f\n"); // report failures
	}
	if(transfer->status != LIBUSB_TRANSFER_CANCELLED) {
		libusb_submit_transfer(transfer); // queue it back up again
		// accumulate feedback on samples required
		unsigned int fSum = 0;
		for(int i=0; i<6; i++) {
			// add up each frequency counter
			fSum += (unsigned int)transfer->buffer[i];
		}
		outDelta += fSum-576;
	}
}

static void bulk_in(struct libusb_transfer *transfer)
{
	//fprintf(stderr,"b");
	int r=0;
	if(transfer->status != LIBUSB_TRANSFER_COMPLETED) {
		logger(1,"!b\n"); // report failures
		r=1;
	}
	if(transfer->status != LIBUSB_TRANSFER_CANCELLED) {
		if(r==0) {
			// process buffer into audio output
			// format: rows of 32 bytes, of which 24 are valid, rest are padding
			// each byte contain 1 bit of a sample, up to 5 samples/byte
			// 2 rows make up all the channel samples for one frame
			// 2048 frames per transfer or 4096 rows
			char *bpos=transfer->buffer; // input byte position in transfer buffer
			int nr = (jack_ringbuffer_write_space(ib) / ibframe)*2; // how many rows of space have we got? Make sure this is a multiple of 2 so we don't drop half a frame..
			
			if (nr<4096) { // overrun! just drop data that does not fit
				logger(1,"\nIN overrun! nr=%d\n",nr);
			} else {
				nr=4096; // process all rows if they fit
			}
			
			// process rows into temp
			float a[10*2048];
			float *ap = a;
			for(int row=0;row<nr; row++) { // step through rows
				// assemble row bits into 5 samples
				int sample[5] = {0,0,0,0,0};
				for(int b=0;b<24;b++) {
					unsigned char bb = *bpos;
					for(int ch=0; ch<5; ch++) {
						sample[ch] <<= 1; // shift sample up one bit
						sample[ch] |= 0x01&bb; // OR channel bit into sample
						bb >>= 1; // shift to next channel bit
					}
					bpos++; // move to next buffer byte
				}
				// convert samples into floats and serialize into temp store
				for(int s=0; s<5; s++) {
					*ap = (sample[s]<<8)/(float)INT_MAX;
					ap++;
				}
				// move transfer buffer point to next row
				bpos+=8;
			}
			// write temp to ring buffer
			if(jack_ringbuffer_write(ib, (void *) a, 5*nr*sample_size)<5*nr*sample_size) {
				logger(1,"\nIN buffer error! QUIT\n"); // this should NOT happen!
				done=1;
			}
			libusb_submit_transfer(transfer); // queue it back up again
		}	
	}
}

static void run_audio(libusb_device_handle *hdev, int epOut, int epInFb, int epInBulk) {
	int r;
	// transfer buffer handles
	unsigned char *bulk[preload];
	unsigned char *fb[preload];
	unsigned char *ob[outpreload];
	// transfer handles
	struct libusb_transfer *transfer_bulk[preload];
	struct libusb_transfer *transfer_fb[preload];
	struct libusb_transfer *transfer_out[outpreload];

	// clear any stalled ports
	r=0;
	logger(0,"clear_halt\n");
	r=libusb_clear_halt(hdev, epOut);
	if(r != 0) { logger(1, libusb_strerror(r)); return;}
	r=libusb_clear_halt(hdev, epInFb);
	if(r != 0) { logger(1, libusb_strerror(r)); return;}
	r=libusb_clear_halt(hdev, epInBulk);
	if(r != 0) { logger(1, libusb_strerror(r)); return;}
	
	// submit a queue of BULK transfers
	for(int i=0; i<preload; i++) {
		bulk[i] = calloc(0x20000,1); // 256 * max packet size (512) = 128kb
		// alloc input transfer struct
		transfer_bulk[i] = libusb_alloc_transfer(0);
		// fill transfer struct data
		libusb_fill_bulk_transfer( transfer_bulk[i], hdev, epInBulk,
		    bulk[i],  0x20000,
		    bulk_in, NULL, 0);
		// submit request
		logger(0,"submit_txfr(b)\n");
		r = libusb_submit_transfer(transfer_bulk[i]);
		if(r != 0) { logger(1, libusb_strerror(r)); return;}
	}	
	
	// submit a queue of ISO FB transfers
	for(int i=0; i<preload; i++) {
		fb[i] = calloc(6,1);	// 2* 3 bytes
		// alloc input transfer struct
		transfer_fb[i] = libusb_alloc_transfer(2); // two ISO packets per tx
		// fill transfer struct data
		libusb_fill_iso_transfer( transfer_fb[i], hdev, epInFb,
		    fb[i],  6, 2,
		    fb_in, NULL, 0);
		libusb_set_iso_packet_lengths(transfer_fb[i],3); // 3 bytes per packet
		// submit request
		logger(0,"submit_txfr(f)\n");
		r = libusb_submit_transfer(transfer_fb[i]);
		if(r != 0) { logger(1, libusb_strerror(r)); return;}
	}
	
	// submit a queue of output transfers - keep it short as this adds latency!
	for(int i=0; i<outpreload; i++) {
		ob[i] = calloc(2886,1); // extra byte for underrun handling
		transfer_out[i] = libusb_alloc_transfer(40);
		// fill transfer struct data
		libusb_fill_iso_transfer( transfer_out[i], hdev, epOut,
		    ob[i],  2880, 40,
		    cb_out, NULL, 0);
		libusb_set_iso_packet_lengths(transfer_out[i],72); // 72 bytes per packet
		// submit request
		logger(0,"submit_txfr(o)\n");
		r = libusb_submit_transfer(transfer_out[i]);
		if(r != 0) { logger(1, libusb_strerror(r)); return;}
	}

	// run processing loop

	// prep for signal capture
	// signal(SIGINT, sig_handler); // CTRL-C to stop - disabled, this does NOT work and core dumps!
	
	// loop processing USB events (jack will callback as required without a loop)
	running=1;
	int cnt=0;
	const int stdinfd = fileno(stdin);
	while(done==0) {
		// check for key press/stdin chars and stop
		int n;
		ioctl(stdinfd, FIONREAD, &n);
		if(n>0) sig_handler(0);
		// blocking API call to poll asynch functions
		//fprintf(stderr,"."); // tracer dots :)
            	r =  libusb_handle_events_completed(ctx, NULL);
		if(r != 0) { logger(1, libusb_strerror(r)); break;}
		if(++cnt>100) { 
			cnt=0;
			fprintf(stderr,"OUT: drop:%08ld add:%08ld fb:%+04d rbdata:%08ld IN: drop:%08ld add:%08ld ibdata:%08.1f\r",
				rbdrop/2, rbadd/2, outDelta, rbavg/rbframe,
				ibdrop/10, ibadd/10, ibavg/ibframe);
		}
	}
	fflush(stdout);
	running=0;
	// cancel transfers and run the loop for another second
	
	logger(0,"Cancelling transfers..\n");
	for(int i=0;i<preload;i++) {
		libusb_cancel_transfer(transfer_bulk[i]);
		libusb_cancel_transfer(transfer_fb[i]);
	}
	for(int i=0; i<outpreload; i++) {
		libusb_cancel_transfer(transfer_out[i]);
	}
	// run the loop again, use non-blocking mode and poll for ~1sec
	cnt=0;
	while(++cnt<1000) {
		// non-blocking API call to poll asynch functions
		//fprintf(stderr,"."); // tracer dots :)
		struct timeval tv;
		tv.tv_usec = 0;
		tv.tv_sec = 0;
            	r =  libusb_handle_events_timeout_completed(ctx, &tv, NULL);
		if(r != 0) { logger(1, libusb_strerror(r)); break;}
		usleep(1000);
	}

	// free memory!
	logger(0,"Free transfers..\n");
	for(int i=0;i<preload;i++) {
		libusb_free_transfer(transfer_bulk[i]);
		free(bulk[i]);
		libusb_free_transfer(transfer_fb[i]);
		free(fb[i]);
	}
	for(int i=0; i<outpreload; i++) {
		libusb_free_transfer(transfer_out[i]);
		free(ob[i]);
	}
}

static libusb_device *find_dev(libusb_device **devs)
{
	libusb_device *dev, *rdev;
	int i = 0, j = 0;
	uint8_t path[8]; 

	rdev = NULL;
	while ((dev = devs[i++]) != NULL) {
		struct libusb_device_descriptor desc;
		int r = libusb_get_device_descriptor(dev, &desc);
		if (r < 0) {
			logger(1, "failed to get device descriptor");
			return rdev;
		}

		logger(0,"%04x:%04x (bus %x, device %x)\n",
			desc.idVendor, desc.idProduct,
			libusb_get_bus_number(dev), libusb_get_device_address(dev));

		r = libusb_get_port_numbers(dev, path, sizeof(path));
		if (r > 0) {
			logger(0," path: %x\n", path[0]);
			for (j = 1; j < r; j++)
				logger(0,".%x", path[j]);
		}
		// check against target IDs
		if(desc.idVendor == targetVendorId && desc.idProduct == targetProductId) {
			logger(0,"found device!\n");
			rdev = dev;
		}
	}
	return rdev;
}


// MAIN //
int main(int argc, char **argv)
{
	const struct libusb_version *ver = libusb_get_version();
	libusb_device **devs, *adev;
	libusb_device_handle *hdev;
	int r;
	ssize_t cnt;
	int tOut[] = targetOutput;
	int tIn[] = targetInput;
	uint16_t ctl1[] = control1;
	unsigned char d1[] = data1;
	uint16_t ctl2[] = control2;
	unsigned char d2[] = data2;
	uint16_t ctl3[] = control3;
	
	const char **ports;
	const char *client_name;
	const char *server_name = NULL;
	jack_options_t options = JackNullOption;
	jack_status_t status;

	// process options
	if(argc<2) { fprintf(stderr,"usage: %s <client name> [-v|-vv]\n",argv[0]); return 0; }
	client_name = argv[1];
	if(argc>2 && strcmp(argv[2],"-v")==0) { debug=1; logger(0,"Debug ON\n"); }
	if(argc>2 && strcmp(argv[2],"-vv")==0) { debug=2; logger(0,"Debug ON, USB debug ON\n"); }
	
	// INIT jack side first - no point opening USB if no jackd!
	/* open a client connection to the JACK server */

	fprintf(stderr,"Starting service: client name: %s\n",client_name);
	client = jack_client_open (client_name, options, &status, server_name);
	if (client == NULL) {
		logger(1, "jack_client_open() failed, "
			 "status = 0x%2.0x\n", status);
		if (status & JackServerFailed) {
			logger(1, "Unable to connect to JACK server\n");
		}
		exit (1);
	}
	if (status & JackServerStarted) {
		logger(0, "JACK server started\n");
	}
	if (status & JackNameNotUnique) {
		client_name = jack_get_client_name(client);
		logger(0, "unique name `%s' assigned\n", client_name);
	}

	/* tell the JACK server to call `process()' whenever
	   there is work to be done.
	*/
	logger(0, "JACK set process callback\n");
	jack_set_process_callback (client, jack_process, NULL);

	/* tell the JACK server to call `jack_shutdown()' if
	   it ever shuts down, either entirely, or if it
	   just decides to stop calling us.
	*/

	logger(0, "JACK set shutdown\n");
	jack_on_shutdown (client, jack_shutdown, 0);

	/* create ten output ports */
	logger(0, "JACK register ports\n");
	char *iname[] = innames;
	for(int i=0; i<10; i++) {
		output_port[i] = jack_port_register (client, iname[i],
					  JACK_DEFAULT_AUDIO_TYPE,
					  JackPortIsOutput | JackPortIsPhysical , 0);
		if(output_port[i] == NULL) {
			logger(1, "no more JACK ports available\n");
			exit (1);
		}
		//jack_port_set_latency(output_port[i],1024);
	}
	// create two input ports
	char *oname[] = outnames;
	for(int i=0; i<2; i++) {
		char name[10];
		sprintf(name,"input%02d",i);
		input_port[i] = jack_port_register (client, oname[i],
					  JACK_DEFAULT_AUDIO_TYPE,
					  JackPortIsInput | JackPortIsPhysical, 0);
		if(input_port[i] == NULL) {
			logger(1, "no more JACK ports available\n");
			exit (1);
		}
		// deprecated - do we even need it?
		//jack_port_set_latency(input_port[i],1024);
	}
	// setup ringbuffer
	// WARNING!! JACK allocates the next highest power of two so these buffers are >= the requested size.
	// DO NOT RELY ON WRITE SPACE for managing latency! use the pointer gap...
	logger(0,"JACK create ring buffer\n");
	rb = jack_ringbuffer_create(rbsize);
	ib = jack_ringbuffer_create(ibsize);
   
	logger(0, "JACK set latency callback\n");
	jack_set_latency_callback (client, jack_latency, NULL);

	// INIT jack end
	
	// INIT Alesis USB audio

//	r = libusb_init_context(/*ctx=*/NULL, /*options=*/NULL, /*num_options=*/0); // not supported yet on Ubuntu 22.04 - revert to old API init
	logger(0, "USB init\n");
	r = libusb_init(&ctx);
	if (r < 0)
		return r;

	// enable debug level 2-4
	logger(0, "USB set debug %d\n",2+debug);
	libusb_set_option(NULL, LIBUSB_OPTION_LOG_LEVEL, 2+debug);

	logger(0,"Using library: %x.%x.%x.%x %s\n\n",ver->major,ver->minor,ver->micro,ver->nano,ver->describe);

	logger(0, "USB get devices\n");
	cnt = libusb_get_device_list(ctx, &devs);
	if (cnt < 0){
		libusb_exit(NULL);
		return (int) cnt;
	}

	adev = find_dev(devs);
	if(adev==NULL) {logger(1,"\nNo target device found\n"); return 1;}
	// get a handle to target device
	logger(0,"USB open\n");
	r = libusb_open(adev, &hdev);
	if(r != 0) { logger(1, libusb_strerror(r));}
	// now free up the list after we used a pointer from it
	logger(0,"USB free_device_list\n");
	libusb_free_device_list(devs, 1);
	// do some munging here...
	//investigate_dev(adev, hdev);
	// open the output endpoint and start send/receive
	// set config 0 then config 1 to reset device
	logger(0,"USB set_configuration 0\n");
	r = libusb_set_configuration(hdev,0);
	if(r != 0) { logger(1, libusb_strerror(r));}
	usleep(10000);	
	logger(0,"USB set_configuration 1\n");
	r = libusb_set_configuration(hdev,1);
	if(r != 0) { logger(1, libusb_strerror(r));}
	// setup kernel driver swapout	
	logger(0,"USB set_auto_detach\n");
	r = libusb_set_auto_detach_kernel_driver(hdev,1);
	if(r != 0) { logger(1, libusb_strerror(r));}
	// get the interfaces
	logger(0,"USB claim_interface(in)\n");
	r = libusb_claim_interface(hdev,tIn[0]);
	if(r != 0) { logger(1, libusb_strerror(r));}
	logger(0,"USB claim_interface(out)\n");
	r = libusb_claim_interface(hdev,tOut[0]);
	if(r != 0) { logger(1, libusb_strerror(r));}
	// set the alt setting = didn't see this in win capture but does not work without it on libusb..
	logger(0,"USB alt_setting(in)\n");
	r = libusb_set_interface_alt_setting(hdev,tIn[0],tIn[1]);
	if(r != 0) { logger(1, libusb_strerror(r));}
	logger(0,"USB alt_setting(out)\n");
	r = libusb_set_interface_alt_setting(hdev,tOut[0],tOut[1]);
	if(r != 0) { logger(1, libusb_strerror(r));}

	// check max packet sizes
	logger(0,"USB maxPkt(o):%x\n",libusb_get_max_iso_packet_size(adev,tOut[2]));
	logger(0,"USB maxPkt(f):%x\n",libusb_get_max_iso_packet_size(adev,tIn[2]));
	logger(0,"USB maxPkt(b):%x\n",libusb_get_max_iso_packet_size(adev,tIn[3]));


	// send vendor controls to set 96k streaming - yes it sends this multiple times! Who knows why... I'm not going to
	for(int i=0;i<ctlRepeat;i++) {
		send_control(hdev,ctl1,d1);
		send_control(hdev,ctl2,d2);
	}
	send_control(hdev,ctl3,NULL); // this is only sent once. Wierd...
	
	// INIT USB end

	logger(0,"Interfaces open! process audio... target RB=%d-%d/%d, target IB=%d-%d/%d\n",
		rbtlow/rbframe,rbthigh/rbframe,rbsize/rbframe,ibtlow/ibframe,ibthigh/ibframe,ibsize/ibframe);

	// start JACK callbacks here

	logger(0, "JACK activate client\n");
	if (jack_activate (client)) {
		logger(1, "cannot activate client");
		exit (1);
	}
	
	// start USB transactions here
		
	run_audio(hdev,tOut[2],tIn[2],tIn[3]);
	
	// cleanup
	logger(0,"USB release_interface(out)\n");
	libusb_release_interface(hdev,tOut[0]);
	logger(0,"USB release_interface(in)\n");
	libusb_release_interface(hdev,tIn[0]);
	logger(0,"USB close device)\n");
	libusb_close(hdev);
	logger(0,"USB close\n");
	libusb_exit(ctx);
	
	logger(0, "JACK cleanup\n");
	jack_ringbuffer_free(rb);
	jack_ringbuffer_free(ib);
	jack_client_close(client);

	return 0;
}
