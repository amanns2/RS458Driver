#include <linux/device.h>
#include <linux/errno.h>
#include <linux/etherdevice.h>
#include <linux/if_arp.h>
#include <linux/if_ether.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_net.h>
#include <linux/sched.h>
#include <linux/serdev.h>
#include <linux/skbuff.h>
#include <linux/types.h>
#include <linux/serial.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/poll.h>

#include "usartNet.h"

#define STMUART_DRV_VERSION "0.1.0"
#define STMUART_DRV_NAME "stmNetUart"
#define STMUART_TX_TIMEOUT (1 * HZ)

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

u16
stmfrm_create_header(u8 *buf, u16 length)
{
	__le16 len;

	if (!buf)
		return 0;

	len = cpu_to_le16(length);

	buf[0] = 0xAA;
	buf[1] = 0xAA;
	buf[2] = 0xAA;
	buf[3] = 0xAA;
	buf[4] = len & 0xff;
	buf[5] = (len >> 8) & 0xff;
	buf[6] = 0;
	buf[7] = 0;

	return STMFRM_HEADER_LEN;
}

u16
stmfrm_create_footer(u8 *buf)
{
	if (!buf)
		return 0;

	buf[0] = 0x55;
	return STMFRM_FOOTER_LEN;
}

s32
stmfrm_fsm_decode(struct stmfrm_handle *handle, u8 *buf, u16 buf_len, u8 recv_byte)
{
	s32 ret = STMFRM_GATHER;
	u16 len;

	switch (handle->state) {
	/* 4 bytes header pattern */
	case STMFRM_WAIT_AA1:
	case STMFRM_WAIT_AA2:
	case STMFRM_WAIT_AA3:
	case STMFRM_WAIT_AA4:
		if (recv_byte != 0xAA) {
			ret = STMFRM_NOHEAD;
			handle->state = handle->init;
		} else {
			handle->state--;
		}
		break;
		/* 2 bytes length. */
		/* Borrow offset field to hold length for now. */
	case STMFRM_WAIT_LEN_BYTE0:
		handle->offset = recv_byte;
		handle->state = STMFRM_WAIT_LEN_BYTE1;
		break;
	case STMFRM_WAIT_LEN_BYTE1:
		handle->offset = handle->offset | (recv_byte << 8);
		handle->state = STMFRM_WAIT_RSVD_BYTE1;
		break;
	case STMFRM_WAIT_RSVD_BYTE1:
		handle->state = STMFRM_WAIT_RSVD_BYTE2;
		break;
	case STMFRM_WAIT_RSVD_BYTE2:
		len = handle->offset;
		if (len > buf_len || len < STMFRM_MIN_LEN) {
			ret = STMFRM_INVLEN;
			handle->state = handle->init;
		} else {
			handle->state = (enum stmfrm_state)(len + 1);
			/* Remaining number of bytes. */
			handle->offset = 0;
		}
		break;
	default:
		/* Receiving Ethernet frame itself. */
		buf[handle->offset] = recv_byte;
		handle->offset++;
		handle->state--;
		break;
	case STMFRM_WAIT_551:
		if (recv_byte != 0x55) {
			ret = STMFRM_NOTAIL;
			handle->state = handle->init;
		} else {
			ret = handle->offset;
			/* Frame is fully received. */
			handle->state = handle->init;
		}
		break;
	}

	return ret;
}

s32
stmfrm_fsm_decode_org(struct stmfrm_handle *handle, u8 *buf, u16 buf_len, u8 recv_byte)
{
	s32 ret = STMFRM_GATHER;
	u16 len;

	switch (handle->state) {
	/* 4 bytes header pattern */
	case STMFRM_WAIT_AA1:
	case STMFRM_WAIT_AA2:
	case STMFRM_WAIT_AA3:
	case STMFRM_WAIT_AA4:
		if (recv_byte != 0xAA) {
			ret = STMFRM_NOHEAD;
			handle->state = handle->init;
		} else {
			handle->state--;
		}
		break;
		/* 2 bytes length. */
		/* Borrow offset field to hold length for now. */
	case STMFRM_WAIT_LEN_BYTE0:
		handle->offset = recv_byte;
		handle->state = STMFRM_WAIT_LEN_BYTE1;
		break;
	case STMFRM_WAIT_LEN_BYTE1:
		handle->offset = handle->offset | (recv_byte << 8);
		handle->state = STMFRM_WAIT_RSVD_BYTE1;
		break;
	case STMFRM_WAIT_RSVD_BYTE1:
		handle->state = STMFRM_WAIT_RSVD_BYTE2;
		break;
	case STMFRM_WAIT_RSVD_BYTE2:
		len = handle->offset;
		if (len > buf_len || len < STMFRM_MIN_LEN) {
			ret = STMFRM_INVLEN;
			handle->state = handle->init;
		} else {
			handle->state = (enum stmfrm_state)(len + 1);
			/* Remaining number of bytes. */
			handle->offset = 0;
		}
		break;
	default:
		/* Receiving Ethernet frame itself. */
		buf[handle->offset] = recv_byte;
		handle->offset++;
		handle->state--;
		break;
	case STMFRM_WAIT_551:
		if (recv_byte != 0x55) {
			ret = STMFRM_NOTAIL;
			handle->state = handle->init;
		} else {
			ret = handle->offset;
			/* Frame is fully received. */
			handle->state = handle->init;
		}
		break;
	}

	return ret;
}

static int
stm_tty_receive(struct serdev_device *serdev, const unsigned char *data,
		size_t count)
{
	struct stmuart *stm = serdev_device_get_drvdata(serdev);
	struct net_device *netdev = stm->net_dev;
	struct net_device_stats *n_stats = &netdev->stats;
	size_t i;

	if (!stm->rx_skb) {
		stm->rx_skb = netdev_alloc_skb_ip_align(netdev,
							netdev->mtu +
							VLAN_ETH_HLEN);
		if (!stm->rx_skb) {
			n_stats->rx_errors++;
			n_stats->rx_dropped++;
			return 0;
		}
	}

	for (i = 0; i < count; i++) {
		s32 retcode;

		retcode = stmfrm_fsm_decode(&stm->frm_handle,
					    stm->rx_skb->data,
					    skb_tailroom(stm->rx_skb),
					    data[i]);

		switch (retcode) {
		case STMFRM_GATHER:
		case STMFRM_NOHEAD:
			break;
		case STMFRM_NOTAIL:
			netdev_dbg(netdev, "recv: no RX tail\n");
			n_stats->rx_errors++;
			n_stats->rx_dropped++;
			break;
		case STMFRM_INVLEN:
			netdev_dbg(netdev, "recv: invalid RX length\n");
			n_stats->rx_errors++;
			n_stats->rx_dropped++;
			break;
		default:
			n_stats->rx_packets++;
			n_stats->rx_bytes += retcode;
			skb_put(stm->rx_skb, retcode);
			stm->rx_skb->protocol = eth_type_trans(
						stm->rx_skb, stm->rx_skb->dev);
			stm->rx_skb->ip_summed = CHECKSUM_NONE;
			netif_rx_ni(stm->rx_skb);
			stm->rx_skb = netdev_alloc_skb_ip_align(netdev,
								netdev->mtu +
								VLAN_ETH_HLEN);
			if (!stm->rx_skb) {
				netdev_dbg(netdev, "recv: out of RX resources\n");
				n_stats->rx_errors++;
				return i;
			}
		}
	}

	return i;
}

/* Write out any remaining transmit buffer. Scheduled when tty is writable */
static void stmuart_transmit(struct work_struct *work)
{
	struct stmuart *stm = container_of(work, struct stmuart, tx_work);
	struct net_device_stats *n_stats = &stm->net_dev->stats;
	int written;

	spin_lock_bh(&stm->lock);

	/* First make sure we're connected. */
	if (!netif_running(stm->net_dev)) {
		spin_unlock_bh(&stm->lock);
		return;
	}
	
	//stm_flow_control(stm->serdev, false);

	if (stm->tx_left <= 0)  {
		/* Now serial buffer is almost free & we can start
		 * transmission of another packet
		 */
		n_stats->tx_packets++;
		spin_unlock_bh(&stm->lock);
		netif_wake_queue(stm->net_dev);
		return;
	}

	written = serdev_device_write_buf(stm->serdev, stm->tx_head,
					  stm->tx_left);
	if (written > 0) {
		stm->tx_left -= written;
		stm->tx_head += written;
	}
	
	//stm_flow_control(stm->serdev, true);
	
	spin_unlock_bh(&stm->lock);
}

/* Called by the driver when there's room for more data.
 * Schedule the transmit.
 */
static void stm_tty_wakeup(struct serdev_device *serdev)
{
	struct stmuart *stm = serdev_device_get_drvdata(serdev);

	schedule_work(&stm->tx_work);
}

static const struct serdev_device_ops stm_serdev_ops = {
	.receive_buf = stm_tty_receive,
	.write_wakeup = stm_tty_wakeup,
};

static int stmuart_netdev_open(struct net_device *dev)
{
	struct stmuart *stm = netdev_priv(dev);

	netif_start_queue(stm->net_dev);
 
	return 0;
}

static int stmuart_netdev_close(struct net_device *dev)
{
	struct stmuart *stm = netdev_priv(dev);

	netif_stop_queue(dev);
	flush_work(&stm->tx_work);

	spin_lock_bh(&stm->lock);
	stm->tx_left = 0;
	spin_unlock_bh(&stm->lock);
	return 0;
}

static netdev_tx_t
stmuart_netdev_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct net_device_stats *n_stats = &dev->stats;
	struct stmuart *stm = netdev_priv(dev);
	u8 pad_len = 0;
	int written;
	u8 *pos;
    
	spin_lock(&stm->lock);

	WARN_ON(stm->tx_left);

	if (!netif_running(dev))  {
		spin_unlock(&stm->lock);
		netdev_warn(stm->net_dev, "xmit: iface is down\n");
		goto out;
	}

	pos = stm->tx_buffer;

	if (skb->len < STMFRM_MIN_LEN)
		pad_len = STMFRM_MIN_LEN - skb->len;

	pos += stmfrm_create_header(pos, skb->len + pad_len);

	memcpy(pos, skb->data, skb->len);
	pos += skb->len;

	if (pad_len) {
		memset(pos, 0, pad_len);
		pos += pad_len;
	}

	pos += stmfrm_create_footer(pos);

	netif_stop_queue(stm->net_dev);

	written = serdev_device_write_buf(stm->serdev, stm->tx_buffer,
					  pos - stm->tx_buffer);
	if (written > 0) {
		stm->tx_left = (pos - stm->tx_buffer) - written;
		stm->tx_head = stm->tx_buffer + written;
		n_stats->tx_bytes += written;
	}
	spin_unlock(&stm->lock);

	netif_trans_update(dev);
out:
	dev_kfree_skb_any(skb);
	return NETDEV_TX_OK;
}

static void stmuart_netdev_tx_timeout(struct net_device *dev, unsigned int txqueue)
{
	struct stmuart *stm = netdev_priv(dev);

	netdev_info(stm->net_dev, "Transmit timeout at %ld, latency %ld\n",
		    jiffies, dev_trans_start(dev));
	dev->stats.tx_errors++;
	dev->stats.tx_dropped++;
}

static int stmuart_netdev_init(struct net_device *dev)
{
	struct stmuart *stm = netdev_priv(dev);
	size_t len;

	/* Finish setting up the device info. */
	dev->mtu = STMFRM_MAX_MTU;
	dev->type = ARPHRD_ETHER;

	len = STMFRM_HEADER_LEN + STMFRM_MAX_LEN + STMFRM_FOOTER_LEN;
	stm->tx_buffer = devm_kmalloc(&stm->serdev->dev, len, GFP_KERNEL);
	if (!stm->tx_buffer)
		return -ENOMEM;

	stm->rx_skb = netdev_alloc_skb_ip_align(stm->net_dev,
						stm->net_dev->mtu +
						VLAN_ETH_HLEN);
	if (!stm->rx_skb)
		return -ENOBUFS;

	return 0;
}

static void stmuart_netdev_uninit(struct net_device *dev)
{
	struct stmuart *stm = netdev_priv(dev);

	dev_kfree_skb(stm->rx_skb);
}

static const struct net_device_ops stmuart_netdev_ops = {
	.ndo_init = stmuart_netdev_init,
	.ndo_uninit = stmuart_netdev_uninit,
	.ndo_open = stmuart_netdev_open,
	.ndo_stop = stmuart_netdev_close,
	.ndo_start_xmit = stmuart_netdev_xmit,
	.ndo_set_mac_address = eth_mac_addr,
	.ndo_tx_timeout = stmuart_netdev_tx_timeout,
	.ndo_validate_addr = eth_validate_addr,
};

static void stmuart_netdev_setup(struct net_device *dev)
{
	dev->netdev_ops = &stmuart_netdev_ops;
	dev->watchdog_timeo = STMUART_TX_TIMEOUT;
	dev->priv_flags &= ~IFF_TX_SKB_SHARING;
	//dev->tx_queue_len = 100;

	/* MTU range: 46 - 1500 */
	dev->min_mtu = STMFRM_MIN_MTU;
	dev->max_mtu = STMFRM_MAX_MTU;
}

static const struct of_device_id stm32_match[] = {
	{
        .compatible = "st,stm32_usart_net",
	},
	{}
};
MODULE_DEVICE_TABLE(of, stm32_match);

static int stm_uart_probe(struct serdev_device *serdev)
{
	struct net_device *stmuart_dev = alloc_etherdev(sizeof(struct stmuart));
	struct stmuart *stm;
	const char *mac;
	u32 speed = 10000000u;
    //u32 speed = 1000000u;
	int ret;
    
	if (!stmuart_dev)
		return -ENOMEM;

	stmuart_netdev_setup(stmuart_dev);
	SET_NETDEV_DEV(stmuart_dev, &serdev->dev);

	stm = netdev_priv(stmuart_dev);
	if (!stm) {
		pr_err("qca_uart: Fail to retrieve private structure\n");
		ret = -ENOMEM;
		goto free;
	}
	stm->net_dev = stmuart_dev;
	stm->serdev = serdev;
	qcafrm_fsm_init_uart(&stm->frm_handle);

	spin_lock_init(&stm->lock);
	INIT_WORK(&stm->tx_work, stmuart_transmit);

	mac = of_get_mac_address(serdev->dev.of_node);

	if (!IS_ERR(mac))
		ether_addr_copy(stm->net_dev->dev_addr, mac);

	//if (!is_valid_ether_addr(stm->net_dev->dev_addr)) {
		eth_hw_addr_random(stm->net_dev);
		dev_info(&serdev->dev, "Using random MAC address: %pM\n",
			 stm->net_dev->dev_addr);
	//}

	netif_carrier_on(stm->net_dev);
	serdev_device_set_drvdata(serdev, stm);
	serdev_device_set_client_ops(serdev, &stm_serdev_ops);
    
	ret = serdev_device_open(serdev);
	if (ret) {
		dev_err(&serdev->dev, "Unable to open device %s\n",
			stmuart_dev->name);
		goto free;
	}

	speed = serdev_device_set_baudrate(serdev, speed);
	dev_info(&serdev->dev, "Using baudrate: %u\n", speed);
   
	serdev_device_set_flow_control(serdev, false);
    
    //serdev_device_set_cts(serdev, false);

	ret = register_netdev(stmuart_dev);
	if (ret) {
		dev_err(&serdev->dev, "Unable to register net device %s\n",
			stmuart_dev->name);
		serdev_device_close(serdev);
		cancel_work_sync(&stm->tx_work);
		goto free;
	}
	
	//serdev_device_set_tiocm(serdev, TIOCM_RTS, 0);
	//ret = serdev_device_set_tiocm(serdev, 0, TIOCM_RTS);
    //ret = serdev_device_get_tiocm(serdev);
    
	return ret;

free:
	free_netdev(stmuart_dev);
	return ret;
}

static void stm_uart_remove(struct serdev_device *serdev)
{
	struct stmuart *stm = serdev_device_get_drvdata(serdev);

	unregister_netdev(stm->net_dev);

	/* Flush any pending characters in the driver. */
	serdev_device_close(serdev);
	cancel_work_sync(&stm->tx_work);

	free_netdev(stm->net_dev);
}

static struct serdev_device_driver stm_uart_driver = {
	.probe = stm_uart_probe,
	.remove = stm_uart_remove,
	.driver = {
		.name = STMUART_DRV_NAME,
		.of_match_table = of_match_ptr(stm32_match),
	},
};

module_serdev_device_driver(stm_uart_driver);

MODULE_DESCRIPTION("Ba USART Network driver");
MODULE_AUTHOR("Qualcomm Atheros Communications");
MODULE_AUTHOR("Sebastian Amann <sebastian.amann@ost.ch>");
MODULE_LICENSE("GPL");
MODULE_VERSION(STMUART_DRV_VERSION);
