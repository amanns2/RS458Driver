#include <linux/if_ether.h>
#include <linux/if_vlan.h>
#include <linux/types.h>
#include <linux/serdev.h>

#define STMUART_DRV_VERSION "0.1.0"
#define STMUART_DRV_NAME "stmNetUart"
#define STMUART_TX_TIMEOUT (1 * HZ)

/* Frame is currently being received */
#define STMFRM_GATHER 0

/*  No header byte while expecting it */
#define STMFRM_NOHEAD (STMFRM_ERR_BASE - 1)

/* Min/Max Ethernet MTU: 46/1500 */
#define STMFRM_MIN_MTU (ETH_ZLEN - ETH_HLEN)
#define STMFRM_MAX_MTU ETH_DATA_LEN

/* Min/Max frame lengths */
#define STMFRM_MIN_LEN (STMFRM_MIN_MTU + ETH_HLEN)
#define STMFRM_MAX_LEN (STMFRM_MAX_MTU + VLAN_ETH_HLEN)


/* Framing. */
#define STMFRM_ERR_BASE -1000

/*  Waiting first 0xAA of header */
#define	STMFRM_WAIT_AA1  0x8000

/*  Waiting second 0xAA of header */
#define	STMFRM_WAIT_AA2 STMFRM_WAIT_AA1 - 1

/*  Waiting third 0xAA of header */ 
#define	STMFRM_WAIT_AA3 STMFRM_WAIT_AA2 - 1

/*  Waiting fourth 0xAA of header */
#define	STMFRM_WAIT_AA4 STMFRM_WAIT_AA3 - 1
    
/*  Waiting fourth 0xAA of header */
#define	STMFRM_WAIT_AA5 STMFRM_WAIT_AA4 - 1
    
/*  Waiting fourth 0xAA of header */
#define	STMFRM_WAIT_AA6 STMFRM_WAIT_AA5 - 1
    
/*  Waiting fourth 0xAA of header */
#define	STMFRM_WAIT_AA7 STMFRM_WAIT_AA6 - 1
    
/*  Waiting fourth 0xAA of header */
#define	STMFRM_WAIT_AB8 STMFRM_WAIT_AA7 - 1


/*   Structure to maintain the frame decoding during reception. */

struct stmfrm_handle {
	/*  Current decoding state */
	int state;
	/* Initial state depends on connection type */
	int init;

	/* Offset in buffer*/
	u16 offset;
};

static const u8 stmfrm_header[8] = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAB};

static inline void qcafrm_fsm_init_uart(struct stmfrm_handle *handle)
{
	handle->init = STMFRM_WAIT_AA1;
	handle->state = handle->init;
}


/*   Gather received bytes and try to extract a full Ethernet frame
 *   by following a simple state machine.
 *
 * Return:   STMFRM_GATHER       No Ethernet frame fully received yet.
 *           STMFRM_NOHEAD       Header expected but not found.
 */

s32 stmfrm_fsm_decode(struct stmfrm_handle *handle, u8 *buf, u8 recv_byte);

struct stmuart {
	struct net_device *net_dev;
	spinlock_t lock;			/* transmit lock */
	struct work_struct tx_work;		/* Flushes transmit buffer   */

	struct serdev_device *serdev;
	struct stmfrm_handle frm_handle;
	struct sk_buff *rx_skb;

	unsigned char *tx_head;			/* pointer to next XMIT byte */
	int tx_left;				/* bytes left in XMIT queue  */
	unsigned char *tx_buffer;
};
