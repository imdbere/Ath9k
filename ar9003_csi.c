/*
 * =====================================================================================
 *       Filename:  ath9k_csi.c
 *
 *    Description:  extrac csi and data together from hardware
 *        Version:  1.0
 *
 *         Author:  Yaxiong Xie 
 *         Email :  <xieyaxiongfly@gmail.com>
 *   Organization:  WANDS group @ Nanyang Technological University 
 *
 *   Copyright (c)  WANDS group @ Nanyang Technological University
 * =====================================================================================
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/wait.h>
#include <linux/netdevice.h>

#include "ar9003_csi.h"
#include "ar9003_mac.h"
#include "ar9003_phy.h"
#include "mac.h"
#include "hw.h"

#define Tx_Buf_LEN                                                             \
	20480 // Generally, buffere with 20480 bits is enough                  \
		// You can change the size freely

//#define Rx_Buf_LEN      128                   // Generally, buffere with 20480 bits is enough

#define BITS_PER_BYTE 8
#define BITS_PER_SYMBOL 10
#define BITS_PER_COMPLEX_SYMBOL (2 * BITS_PER_SYMBOL)

#define DEVICE_NAME "CSI_dev"
#define CLASS_NAME "CSI_class"
#define AH_MAX_CHAINS 3 //maximum chain number, we set it to 3
#define NUM_OF_CHAINMASK (1 << AH_MAX_CHAINS)

volatile u32 csi_head;
volatile u32 csi_tail;
volatile u32 csi_len;
volatile u32 csi_valid;
volatile u32 recording;

static struct ath9k_csi csi_buf[16]; //TODO
static char tx_buf[Tx_Buf_LEN];
//static char         rx_buf[Rx_Buf_LEN];

static int majorNumber;
static struct class *ebbcharClass = NULL;
static struct device *ebbcharDevice = NULL;

DECLARE_WAIT_QUEUE_HEAD(csi_queue);

static int csi_open(struct inode *inode, struct file *file);
static int csi_close(struct inode *inode, struct file *file);
static ssize_t csi_read(struct file *file, char __user *user_buf, size_t count,
			loff_t *ppos);
static ssize_t csi_write(struct file *file, const char __user *user_buf,
			 size_t count, loff_t *ppos);

//register module functions
static const struct file_operations csi_fops = {
	.read = csi_read,
	.write = csi_write,
	.open = csi_open,
	.release = csi_close,
	.llseek = default_llseek,
};

static u_int8_t Num_bits_on[NUM_OF_CHAINMASK] = {
	0 /*       000 */, 1 /*       001 */,
	1 /*       010 */, 2 /*       011 */,
	1 /*       100 */, 2 /*       101 */,
	2 /*       110 */, 3 /*       111 */
};

u_int8_t ar9300_get_nrx_csi(struct ath_hw *ah)
{
	return Num_bits_on[ah->rxchainmask];
}

static int __init csi_init(void)
{
	// initalize parameters
	csi_head = 0;
	csi_tail = 0;
	recording = 0;
	csi_valid = 0;

	// Try to dynamically allocate a major number for the device -- more difficult but worth it
	majorNumber = register_chrdev(0, DEVICE_NAME, &csi_fops);
	if (majorNumber < 0) {
		printk(KERN_ALERT
		       "debug_csi: failed to register a major number\n");
		return majorNumber;
	}
	printk(KERN_INFO
	       "debug_csi: registered correctly with major number %d\n",
	       majorNumber);

	// Register the device class
	ebbcharClass = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(ebbcharClass)) { // Check for error and clean up if there is
		unregister_chrdev(majorNumber, DEVICE_NAME);
		printk(KERN_ALERT
		       "debug_csi: Failed to register device class\n");
		return PTR_ERR(
			ebbcharClass); // Correct way to return an error on a pointer
	}
	printk(KERN_INFO "debug_csi: device class registered correctly\n");

	// Register the device driver
	ebbcharDevice = device_create(ebbcharClass, NULL, MKDEV(majorNumber, 0),
				      NULL, DEVICE_NAME);
	if (IS_ERR(ebbcharDevice)) { // Clean up if there is an error
		class_destroy(
			ebbcharClass); // Repeated code but the alternative is goto statements
		unregister_chrdev(majorNumber, DEVICE_NAME);
		printk(KERN_ALERT "Failed to create the device\n");
		return PTR_ERR(ebbcharDevice);
	}
	printk(KERN_INFO
	       "debug_csi: device class created correctly \n"); // Made it! device was initialized

	return 0;
}

static void __exit csi_exit(void)
{
	/* delete and unregister the devices we have created and registered */
	device_destroy(ebbcharClass,
		       MKDEV(majorNumber, 0)); // remove the device
	class_unregister(ebbcharClass); // unregister the device class
	class_destroy(ebbcharClass); // remove the device class
	unregister_chrdev(majorNumber,
			  DEVICE_NAME); // unregister the major number
	printk(KERN_INFO "debug_csi: Goodbye CSI device!\n");
}

static int csi_open(struct inode *inode, struct file *file)
{
	printk(KERN_ALERT "debug_csi: csi open! \n");
	recording = 1; // we can begin to record when
		// the devices is open
	return 0;
}

static int csi_close(struct inode *inode, struct file *file)
{
	printk(KERN_ALERT "debug_csi: csi close! \n");
	recording = 0; // close and reset
	return 0;
}

//read csi and received data
static ssize_t csi_read(struct file *file, char __user *user_buf, size_t count,
			loff_t *ppos)
{
	u_int16_t len;
	u_int8_t *csi_buf_addr;
	u_int8_t *payload_buf_addr;
	u_int16_t csi_len, payload_len;
	struct ath9k_csi *csi;
	struct csi_pkt_status *RxStatus;

	*ppos = 0;

	if (csi_head == csi_tail) { // wait until time out
		wait_event_interruptible_timeout(csi_queue,
						 csi_head != csi_tail, 5 * HZ);
	}
	if (csi_head != csi_tail) {
		csi = (struct ath9k_csi *)&csi_buf[csi_tail];
		len = 0;

		RxStatus = &(csi->pkt_status); // the status struct

		csi_len = RxStatus->csi_len; // csi length (bytes)
		csi_buf_addr = csi->csi_buf; // csi buffer
		payload_len = csi->payload_len; // payload length (bytes)
		payload_buf_addr = csi->payload_buf; // payload buffer

		//1. rx_status
		memcpy(tx_buf, RxStatus, 23); // copy the status to the buffer
		len += 23;

		//2. payload len, this is stored in the rx status
		memcpy(tx_buf + len, &payload_len,
		       2); // record the length of payload
		len += 2;

		//3. csi (csi len is stored in the rx status)
		if (csi_len > 0) {
			memcpy(tx_buf + len, csi_buf_addr,
			       csi_len); // copy csi to the buffer
			len += csi_len;
		}
		//4. payload
		memcpy(tx_buf + len, payload_buf_addr,
		       payload_len); // copy payload to the buffer
		len += payload_len;

		//5. total len, used for double checking?
		memcpy(tx_buf + len, &len, 2); // record how many bytes we copy
		len += 2;

		copy_to_user(user_buf, tx_buf, len); // COPY

		csi_tail = (csi_tail + 1) & 0x0000000F; // shift to next buffer
		return len;
	} else {
		return 0;
	}
}

static ssize_t csi_write(struct file *file, const char __user *user_buf,
			 size_t count, loff_t *ppos)
{
	printk(KERN_ALERT "debug_csi: csi write!\n");
	return 0;
}

//payload buffer, the transmitted data
void csi_record_payload(void *data, u_int16_t data_len)
{
	printk(KERN_INFO "recording csi payload");
	struct ath9k_csi *csi;
	if (recording) {
		if (((csi_head + 1) & 0x0000000F) ==
		    csi_tail) // check and update
			csi_tail = (csi_tail + 1) & 0x0000000F;

		csi = (struct ath9k_csi *)&csi_buf[csi_head];
		memcpy((void *)(csi->payload_buf), data,
		       data_len); // copy the payload
		csi->payload_len =
			data_len; // record the payload length (bytes)
		csi_valid = 1;
	}
}
EXPORT_SYMBOL(csi_record_payload);

void convert10To8Bit(int16_t numbers[], int count, uint8_t* buff);

/*void convert10To8Bit(int16_t numbers[], uint8_t* buff) {
	unsigned int i, j;
    uint16_t* posNumbers = (uint16_t*) numbers;

	for(i=0; i<sizeof(posNumbers); i+=5) {
		unsigned long long x=0;
		for (j=0; j<4; j++) {
			x |= (posNumbers[i+j] & 0x03FF) << (10*j);
		}

		for (j=0; j<5; j++) {
			buff[i+j] = (x >> (j*8)) & 0xFF;
		}
	}
}*/

void convert10To8Bit(int16_t numbers[], int count, u_int8_t* buff) {
    uint32_t currentNumber = 0;
    uint16_t* posNumbers = (uint16_t*) numbers;
    u_int8_t remainingBits = 0;
    int currentNumberIndex = 0;

	int i;
    for (i = 0; currentNumberIndex <= count; i++)
    {
        if (remainingBits < 10) {
            uint16_t newNumber = currentNumberIndex < count ?
                (posNumbers[currentNumberIndex] & 0x03FF) : 0;

            currentNumberIndex++;
            currentNumber += newNumber << remainingBits;
            remainingBits += 10;
        }

        u_int8_t currByte = (uint8_t) (currentNumber & 0xFF);
        buff[i] = currByte;
        
        remainingBits -= 8;
        currentNumber = currentNumber >> 8;
    }
}

void csi_record_status_dummy(struct ath_hw* ah, struct ath_rx_status* rx_status, int16_t numbers[], int numbersCount) {
	u_int8_t buffer[2000];
	convert10To8Bit(numbers, numbersCount, (u_int8_t*) (&buffer));

	struct ath_hw ah_dummy;
	memcpy((void *) (&ah_dummy), (void *) ah, sizeof(struct ath_hw));

	struct ath_rx_status rx_status_dummy;
	memcpy((void *) (&rx_status_dummy), (void *) rx_status, sizeof(struct ath_rx_status));

	rx_status_dummy.rs_datalen = (numbersCount + (8-1)) / 8 * 10;
	printk(KERN_INFO "CSI data length is %i", rx_status_dummy.rs_datalen);
	// 2 RX antennas
	ah_dummy.rxchainmask = 3;

	struct ar9003_rxs rxsp = {{0}};
	rxsp.status2 = AR_hw_upload_data;
	rxsp.status4 = AR_hw_upload_data_valid;// | AR_2040;
	rxsp.status11 = 0x02000000;

	csi_valid = true;
	csi_record_status(&ah_dummy, &rx_status_dummy, &rxsp, (void *) buffer);
}

EXPORT_SYMBOL(csi_record_status_dummy);

//csi status, csi information,
//output: data && rxs
//tansmitted data is not stored in rxs
//this function is used in ar9003_mac.c, then the data is stored in csi_buf
void csi_record_status(struct ath_hw *ah, struct ath_rx_status *rxs,
		       struct ar9003_rxs *rxsp, void *data)
{
	printk(KERN_INFO "recording csi status");
	struct ath9k_csi *csi;

	u_int8_t nr;
	u_int8_t chan_BW;
	u_int8_t rx_not_sounding;
	u_int8_t rx_hw_upload_data;
	u_int8_t rx_hw_upload_data_valid;
	u_int8_t rx_hw_upload_data_type;

	rx_hw_upload_data = (rxsp->status2 & AR_hw_upload_data) ? 1 : 0;
	// What is this good for ?
	rx_not_sounding = (rxsp->status4 & AR_rx_not_sounding) ? 1 : 0;
	rx_hw_upload_data_valid =
		(rxsp->status4 & AR_hw_upload_data_valid) ? 1 : 0;
	rx_hw_upload_data_type = MS(rxsp->status11, AR_hw_upload_data_type);

	printk(KERN_INFO "upload_data: %i, not_sounding: %i, data_valid: %i, data_type: %i", rx_hw_upload_data, rx_not_sounding, rx_hw_upload_data_valid, rx_hw_upload_data_type);
	if (rxs->rs_phyerr == 0 && rx_hw_upload_data == 0 &&
	    rx_hw_upload_data_valid == 0 && rx_hw_upload_data_type == 0) {
		printk(KERN_INFO "no valid csi data to process");
		return;
	}

	if (recording && csi_valid == 1) {
		printk(KERN_INFO "now recording csi status");
		csi = (struct ath9k_csi *)&csi_buf[csi_head];

		csi->pkt_status.tstamp =
			rxs->rs_tstamp; // time stamp of the rx packet

		csi->pkt_status.channel = ah->curchan->channel;

		chan_BW = (rxsp->status4 & AR_2040) >> 1;
		csi->pkt_status.ChanBW = chan_BW; // channel bandwidth
		nr = ar9300_get_nrx_csi(ah);
		csi->pkt_status.nr = nr; // rx antennas number

		csi->pkt_status.phyerr = rxs->rs_phyerr; // PHY layer error code

		csi->pkt_status.rssi = rxs->rs_rssi;
		csi->pkt_status.rssi_ctl0 = rxs->rs_rssi_ctl[0];
		csi->pkt_status.rssi_ctl1 = rxs->rs_rssi_ctl[1];
		csi->pkt_status.rssi_ctl2 = rxs->rs_rssi_ctl[2];

		csi->pkt_status.noise = 0; // to be updated
		csi->pkt_status.rate = rxs->rs_rate; // data rate

		rx_hw_upload_data = (rxsp->status2 & AR_hw_upload_data) ? 1 : 0;
		rx_not_sounding = (rxsp->status4 & AR_rx_not_sounding) ? 1 : 0;
		rx_hw_upload_data_valid =
			(rxsp->status4 & AR_hw_upload_data_valid) ? 1 : 0;
		rx_hw_upload_data_type =
			MS(rxsp->status11, AR_hw_upload_data_type);

		// Decides how many tones(subcarriers) are used according to the channel bandwidth
		if (chan_BW == 0) {
			csi->pkt_status.num_tones = 56; // 20MHz Channel
		} else if (chan_BW == 1) {
			csi->pkt_status.num_tones = 114; // 40MHz Channel
		} else {
			csi->pkt_status.num_tones = 56; // 20MHz Channel
			printk("Error happends for channel bandwidth recording!!\n");
		}

		/* tx antennas number 
         * NOTE: when the packet is received with error
         * The antenna number value is not correct
         */
		csi->pkt_status.nc =
			(int)(rxs->rs_datalen * BITS_PER_BYTE) /
			(int)(BITS_PER_COMPLEX_SYMBOL * csi->pkt_status.nr *
			      csi->pkt_status.num_tones);
		printk("debug_csi: nr is: %d, nc is %d   \n\n",
		       csi->pkt_status.nr, csi->pkt_status.nc);
		/* copy the csi value to the allocated csi buffer */
		printk(KERN_INFO "upload_data %i, data_valid %i, data_type %i", rx_hw_upload_data, rx_hw_upload_data_valid, rx_hw_upload_data_type);
		if (rxs->rs_datalen > 0 && rx_hw_upload_data == 1 &&
		    rx_hw_upload_data_valid == 1 &&
		    rx_hw_upload_data_type == 1) {
			csi->pkt_status.csi_len = rxs->rs_datalen;
			memcpy((void *)(csi->csi_buf), data, rxs->rs_datalen);
			printk(KERN_INFO "csi data registered!");
		} else {
			printk(KERN_INFO "no csi data registered");
			csi->pkt_status.csi_len = 0;
		}

		csi_valid = 0; // update
		csi_head = (csi_head + 1) & 0x0000000F;

		wake_up_interruptible(&csi_queue); // wake up waiting queue
	}
}
EXPORT_SYMBOL(csi_record_status);


void csi_record_status_2(struct ath_hw *ah, struct ath_rx_status *rxs,
		       struct ar5416_desc *ads, void *data)
{
	printk(KERN_INFO "recording csi status");
	struct ath9k_csi *csi;

	u_int8_t nr;
	u_int8_t chan_BW;
	u_int8_t rx_not_sounding;
	u_int8_t rx_hw_upload_data;
	//u_int8_t rx_hw_upload_data_valid;
	u_int8_t rx_hw_upload_data_type;

	// Not in descriptor
	//rx_hw_upload_data = (rxsp->status2 & AR_hw_upload_data) ? 1 : 0;
	// Not in desc either
	//rx_not_sounding = (rxsp->status4 & AR_rx_not_sounding) ? 1 : 0;

	//rx_hw_upload_data_valid =	(rxsp->status4 & AR_hw_upload_data_valid) ? 1 : 0;
	//rx_hw_upload_data_type = MS(rxsp->status11, AR_hw_upload_data_type);

	// rxs->rs_phyerr == 0 means no errors, why would we exclude valid data ?
	/* if (rxs->rs_phyerr == 0) {
		printk(KERN_INFO "no valid csi data to process");
		return;
	} */

	if (recording /*&& csi_valid == 1*/) {
		printk(KERN_INFO "now recording csi status");
		csi = (struct ath9k_csi *)&csi_buf[csi_head];

		csi->pkt_status.tstamp =
			rxs->rs_tstamp; // time stamp of the rx packet

		csi->pkt_status.channel = ah->curchan->channel;

		chan_BW = (ads->ds_rxstatus7 & AR_2040) >> 1;
		csi->pkt_status.ChanBW = chan_BW; // channel bandwidth
		nr = ar9300_get_nrx_csi(ah);
		csi->pkt_status.nr = nr; // rx antennas number

		csi->pkt_status.phyerr = rxs->rs_phyerr; // PHY layer error code

		csi->pkt_status.rssi = rxs->rs_rssi;
		csi->pkt_status.rssi_ctl0 = rxs->rs_rssi_ctl[0];
		csi->pkt_status.rssi_ctl1 = rxs->rs_rssi_ctl[1];
		csi->pkt_status.rssi_ctl2 = rxs->rs_rssi_ctl[2];

		csi->pkt_status.noise = 0; // to be updated
		csi->pkt_status.rate = rxs->rs_rate; // data rate

		/*rx_hw_upload_data = (rxsp->status2 & AR_hw_upload_data) ? 1 : 0;
		rx_not_sounding = (rxsp->status4 & AR_rx_not_sounding) ? 1 : 0;
		rx_hw_upload_data_valid =
			(rxsp->status4 & AR_hw_upload_data_valid) ? 1 : 0;
		rx_hw_upload_data_type =
			MS(rxsp->status11, AR_hw_upload_data_type);*/

		// Decides how many tones(subcarriers) are used according to the channel bandwidth
		if (chan_BW == 0) {
			csi->pkt_status.num_tones = 56; // 20MHz Channel
		} else if (chan_BW == 1) {
			csi->pkt_status.num_tones = 114; // 40MHz Channel
		} else {
			csi->pkt_status.num_tones = 56; // 20MHz Channel
			printk("Error happends for channel bandwidth recording!!\n");
		}

		/* tx antennas number 
         * NOTE: when the packet is received with error
         * The antenna number value is not correct
         */
		csi->pkt_status.nc =
			(int)(rxs->rs_datalen * BITS_PER_BYTE) /
			(int)(BITS_PER_COMPLEX_SYMBOL * csi->pkt_status.nr *
			      csi->pkt_status.num_tones);
		printk("debug_csi: nr is: %d, nc is %d   \n\n",
		       csi->pkt_status.nr, csi->pkt_status.nc);
		/* copy the csi value to the allocated csi buffer */
		//printk(KERN_INFO "upload_data %i, data_valid %i, data_type %i", rx_hw_upload_data, rx_hw_upload_data_valid, rx_hw_upload_data_type);
		if (rxs->rs_datalen > 0/* && rx_hw_upload_data == 1 &&
		    rx_hw_upload_data_valid == 1 &&
		    rx_hw_upload_data_type == 1*/) {
			csi->pkt_status.csi_len = rxs->rs_datalen;
			memcpy((void *)(csi->csi_buf), data, rxs->rs_datalen);
			printk(KERN_INFO "csi data registered!");
		} else {
			printk(KERN_INFO "no csi data registered");
			csi->pkt_status.csi_len = 0;
		}

		csi_valid = 0; // update
		csi_head = (csi_head + 1) & 0x0000000F;

		wake_up_interruptible(&csi_queue); // wake up waiting queue
	}
}
EXPORT_SYMBOL(csi_record_status_2);


module_init(csi_init);
module_exit(csi_exit);

MODULE_AUTHOR("YAXIONG XIE");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("CSI EXTRACTION");
