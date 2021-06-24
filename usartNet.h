#include <linux/if_ether.h>
#include <linux/if_vlan.h>
#include <linux/types.h>
#include <linux/serdev.h>
#include <linux/gpio/consumer.h>

#define STMUART_DRV_VERSION "0.1.0"
#define STMUART_DRV_NAME "stmNetUart"
#define STMUART_TX_TIMEOUT (1 * HZ)

/* Frame is currently being received */
#define STMFRM_GATHER 0

/*  No header byte while expecting it */
#define STMFRM_NOHEAD (STMFRM_ERR_BASE - 1)

/* No tailer byte while expecting it */
#define STMFRM_NOTAIL (STMFRM_ERR_BASE - 2)

/* Frame length is invalid */
#define STMFRM_INVLEN (STMFRM_ERR_BASE - 3)

/* Frame length is invalid */
#define STMFRM_INVFRAME (STMFRM_ERR_BASE - 4)

/* Min/Max Ethernet MTU: 46/1500 */
#define STMFRM_MIN_MTU (ETH_ZLEN - ETH_HLEN)
#define STMFRM_MAX_MTU ETH_DATA_LEN

/* Min/Max frame lengths */
#define STMFRM_MIN_LEN (STMFRM_MIN_MTU + ETH_HLEN)
#define STMFRM_MAX_LEN (STMFRM_MAX_MTU + VLAN_ETH_HLEN)

/* QCA7K header len */
#define STMFRM_HEADER_LEN 8

/* QCA7K footer len */
#define STMFRM_FOOTER_LEN 2

/* QCA7K Framing. */
#define STMFRM_ERR_BASE -1000

enum stmfrm_state {
	/*  Waiting first 0xAA of header */
	STMFRM_WAIT_AA1 = 0x8000,

	/*  Waiting second 0xAA of header */
	STMFRM_WAIT_AA2 = STMFRM_WAIT_AA1 - 1,

	/*  Waiting third 0xAA of header */ 
	STMFRM_WAIT_AA3 = STMFRM_WAIT_AA2 - 1,

	/*  Waiting fourth 0xAA of header */
	STMFRM_WAIT_AA4 = STMFRM_WAIT_AA3 - 1,

	/*  Waiting Byte 0-1 of length (litte endian) */
	STMFRM_WAIT_LEN_BYTE0 = STMFRM_WAIT_AA4 - 1,
	STMFRM_WAIT_LEN_BYTE1 = STMFRM_WAIT_AA4 - 2,

	/* Reserved bytes */
	STMFRM_WAIT_RSVD_BYTE1 = STMFRM_WAIT_AA4 - 3,
	STMFRM_WAIT_RSVD_BYTE2 = STMFRM_WAIT_AA4 - 4,

	/*  The frame length is used as the state until
	 *  the end of the Ethernet frame
	 *  Waiting for first 0x55 of footer
	 */
	STMFRM_WAIT_551 = 1,
};

/*   Structure to maintain the frame decoding during reception. */

struct stmfrm_handle {
	/*  Current decoding state */
	enum stmfrm_state state;
	/* Initial state depends on connection type */
	enum stmfrm_state init;

	/* Offset in buffer (borrowed for length too) */
	u16 offset;

	/* Frame length as kept by this module */
	u16 len;
};

u16 stmfrm_create_header(u8 *buf, u16 len);

u16 stmfrm_create_footer(u8 *buf);

static inline void qcafrm_fsm_init_uart(struct stmfrm_handle *handle)
{
	handle->init = STMFRM_WAIT_AA1;
	handle->state = handle->init;
}


/*   Gather received bytes and try to extract a full Ethernet frame
 *   by following a simple state machine.
 *
 * Return:   QCAFRM_GATHER       No Ethernet frame fully received yet.
 *           QCAFRM_NOHEAD       Header expected but not found.
 *           QCAFRM_INVLEN       QCA7K frame length is invalid
 *           QCAFRM_NOTAIL       Footer expected but not found.
 *           > 0                 Number of byte in the fully received
 *                               Ethernet frame
 */

s32 stmfrm_fsm_decode(struct stmfrm_handle *handle, u8 *buf, u16 buf_len, u8 recv_byte);

struct stmuart {
	struct net_device *net_dev;
    struct gpio_desc *rts_gpio;
	spinlock_t lock;			/* transmit lock */
	struct work_struct tx_work;		/* Flushes transmit buffer   */

	struct serdev_device *serdev;
	struct stmfrm_handle frm_handle;
	struct sk_buff *rx_skb;

	unsigned char *tx_head;			/* pointer to next XMIT byte */
	int tx_left;				/* bytes left in XMIT queue  */
	unsigned char *tx_buffer;
};
