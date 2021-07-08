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
#include <linux/delay.h>

#include "serdevNetwork.h"

s32
stmfrm_fsm_decode(struct stmfrm_handle *handle, u8 *buf, u8 recv_byte)
{
	s32 ret = STMFRM_GATHER;

	switch (handle->state) {
	/* 4 bytes header pattern */
	case STMFRM_WAIT_AA1:
	case STMFRM_WAIT_AA2:
	case STMFRM_WAIT_AA3:
	case STMFRM_WAIT_AA4:
    case STMFRM_WAIT_AA5:
    case STMFRM_WAIT_AA6:
    case STMFRM_WAIT_AA7:
		if (recv_byte != 0xAA) {
			ret = STMFRM_NOHEAD;
			handle->state = handle->init;
		} else {
			handle->state--;
		}
		break;
    case STMFRM_WAIT_AB8:
		if (recv_byte != 0xAB) {
			ret = STMFRM_NOHEAD;
			handle->state = handle->init;
		} else {
            handle->offset = 0;
			handle->state--;
		}
		break;
	default:
		/* Receiving Ethernet frame itself. */
		buf[handle->offset] = recv_byte;
		handle->offset++;
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
    struct stmfrm_handle *frame_handle = &stm->frm_handle;
	size_t i, c;

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
	
	if(count > (netdev->mtu + VLAN_ETH_HLEN)){
        c = (netdev->mtu + VLAN_ETH_HLEN);
    }else{
        c = count;
    }

	for (i = 0; i < c; i++) {
		s32 retcode;

		retcode = stmfrm_fsm_decode(frame_handle,
					    stm->rx_skb->data,
					    data[i]);

		switch (retcode) {
		case STMFRM_GATHER:
		case STMFRM_NOHEAD:
            break;
		}
	}
	
	frame_handle->state = frame_handle->init;
	n_stats->rx_packets++;
    n_stats->rx_bytes += frame_handle->offset;
    skb_put(stm->rx_skb, frame_handle->offset);
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

	memcpy(pos, stmfrm_header, sizeof stmfrm_header);
    pos += sizeof stmfrm_header;

	memcpy(pos, skb->data, skb->len);
	pos += skb->len;

	if (pad_len) {
		memset(pos, 0, pad_len);
		pos += pad_len;
	}

	netif_stop_queue(stm->net_dev);

	written = serdev_device_write_buf(stm->serdev, stm->tx_buffer,
					  pos - stm->tx_buffer);
    
	if (written > 0) {
		stm->tx_left = (pos - stm->tx_buffer) - written;
		stm->tx_head = stm->tx_buffer + written;
		n_stats->tx_bytes += written;
	}
	

out:
    spin_unlock(&stm->lock);
    netif_trans_update(dev);
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
 	dev->type = ARPHRD_IEEE802;

	len = (sizeof stmfrm_header) + STMFRM_MAX_LEN;
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
	dev->tx_queue_len = 100;

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
	//u32 speed = 10000000u;
    u32 speed = 3000000u;
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

	ret = register_netdev(stmuart_dev);
	if (ret) {
		dev_err(&serdev->dev, "Unable to register net device %s\n",
			stmuart_dev->name);
		serdev_device_close(serdev);
		cancel_work_sync(&stm->tx_work);
		goto free;
	}
    
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
MODULE_AUTHOR("Sebastian Amann <sebastian.amann@ost.ch>");
MODULE_LICENSE("GPL");
MODULE_VERSION(STMUART_DRV_VERSION);
