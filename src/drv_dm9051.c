#include <stdint.h>
#include <stdlib.h>

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>

#include "drv_dm9051.h"

#include <netif/ethernetif.h>

//#define DM9051_RX_DUMP
//#define DM9051_TX_DUMP
//#define DM9051_DUMP_RAW

//#define DM9051_FLOWCONTROL_EN
//#define DM9051_WAIT_NWAY_LINK_EN
//#define DM9051_PACKET_CNT_EN
#define DM9051_NO_PBUF_LEVEL  1

#define DBG_SECTION_NAME    "[drv.dm9051] "
#define DBG_LEVEL           DBG_INFO
#define DBG_COLOR
#include "rtdbg.h"

#define MAX_ADDR_LEN 6
struct rt_dm9051_eth
{
    /* inherit from ethernet device */
    struct eth_device parent;

    struct rt_spi_device *spi_device;
    uint8_t rst_pin, int_pin;
    uint32_t irq_count;

    /* interface address info. */
    rt_uint8_t  dev_addr[MAX_ADDR_LEN]; /* hw address */

    struct rt_semaphore tx_buf_free;
    struct rt_mutex lock;
    struct rt_timer timer;

#if (LWIP_IPV4 && LWIP_IGMP) || (LWIP_IPV6 && LWIP_IPV6_MLD)
    uint32_t HashTableHigh;
    uint32_t HashTableLow;
#endif

#ifdef DM9051_PACKET_CNT_EN
    uint32_t tx_count;
    uint32_t rx_count;
#endif /* DM9051_PACKET_CNT_EN */
};

#define dm9051_lock(dev)      rt_mutex_take(&((struct rt_dm9051_eth*)dev)->lock, RT_WAITING_FOREVER);
#define dm9051_unlock(dev)    rt_mutex_release(&((struct rt_dm9051_eth*)dev)->lock);

#if defined(DM9051_RX_DUMP) ||  defined(DM9051_TX_DUMP)
static void packet_dump(const char * msg, const struct pbuf* p)
{
#ifdef DM9051_DUMP_RAW    
    const struct pbuf* q;
    rt_uint32_t i,j;
    rt_uint8_t *ptr;

    rt_kprintf("%s %d byte\n", msg, p->tot_len);

    i=0;
    for(q=p; q != RT_NULL; q= q->next)
    {
        ptr = q->payload;

        for(j=0; j<q->len; j++)
        {
            if( (i%8) == 0 )
            {
                rt_kprintf("  ");
            }
            if( (i%16) == 0 )
            {
                rt_kprintf("\r\n");
            }
            rt_kprintf("%02X ", *ptr);

            i++;
            ptr++;
        }
    }

    rt_kprintf("\n\n");
#else /* DM9051_DUMP_RAW */
    rt_uint8_t header[6 + 6 + 2];
    rt_uint16_t type;

    pbuf_copy_partial(p, header, sizeof(header), 0);
    type = (header[12] << 8) | header[13];

    rt_kprintf("%02X:%02X:%02X:%02X:%02X:%02X <== %02X:%02X:%02X:%02X:%02X:%02X ",
               header[0], header[1], header[2], header[3], header[4], header[5],
               header[6], header[7], header[8], header[9], header[10], header[11]);

    switch (type)
    {
    case 0x0800:
        rt_kprintf("IPv4. ");
        break;

    case 0x0806:
        rt_kprintf("ARP.  ");
        break;

    case 0x86DD:
        rt_kprintf("IPv6. ");
        break;

    default:
        rt_kprintf("%04X. ", type);
        break;
    }

    rt_kprintf("%s %d byte. \n", msg, p->tot_len);
#endif /* DM9051_DUMP_RAW */
}
#else
#define packet_dump(...)
#endif /* dump */

static uint8_t DM9051_read_reg(struct rt_spi_device *spi_device, uint8_t reg_offsetset)
{
    uint8_t reg_value;

    rt_spi_send_then_recv(spi_device, &reg_offsetset, 1, &reg_value, 1);

    return reg_value;
}

static void DM9051_write_reg(struct rt_spi_device *spi_device, uint8_t reg_offset, uint8_t reg_value)
{
	uint8_t buf[2];

	buf[0] = (reg_offset | DM_SPI_WR);
	buf[1] = reg_value;

    rt_spi_send(spi_device, buf, 2);

	return;
}

static void DM9051_read_mem(struct rt_spi_device *spi_device, uint8_t *buf, uint32_t len)
{
    uint8_t cmd = DM_SPI_RD | DM_SPI_MRCMD; // SPI_RD_BURST = 0x72

    rt_spi_send_then_recv(spi_device, &cmd, 1, buf, len);
}

static void DM9051_write_mem(struct rt_spi_device *spi_device, const void *buf, uint32_t len)
{
    uint8_t cmd = DM_SPI_WR | DM_SPI_MWCMD; // SPI_WR_BURST = 0xF8

    rt_spi_send_then_send(spi_device, &cmd, 1, buf, len);
}

static void dm9051_soft_reset(struct rt_spi_device *spi_device)
{
    rt_thread_delay(2); // delay 2 ms any need before NCR_RST (20170510)
    DM9051_write_reg(spi_device, DM9051_NCR, DM9051_NCR_REG_RESET);
    rt_thread_delay(2);
    DM9051_write_reg(spi_device, DM9051_NCR, 0);

    /* Setup DM9051 Registers */
    DM9051_write_reg(spi_device, DM9051_NCR, NCR_DEFAULT);
    DM9051_write_reg(spi_device, DM9051_IMR, DM9051_IMR_OFF);
    DM9051_write_reg(spi_device, DM9051_TCR, TCR_DEFAULT);
    DM9051_write_reg(spi_device, DM9051_BPTR, BPTR_DEFAULT);
    DM9051_write_reg(spi_device, DM9051_FCTR, FCTR_DEAFULT);
    DM9051_write_reg(spi_device, DM9051_FCR, FCR_DEFAULT);

    //DM9051_write_reg(spi_device, DM9051_INTCR, (0<<1) | (1<<0)); /* [1] 0:push-pull. [0] 0:active high, 1:active low. */
    DM9051_write_reg(spi_device, DM9051_INTCR, 0x00);
    DM9051_write_reg(spi_device, DM9051_INTCKCR, 0x81);

    /* Clear status */
    DM9051_write_reg(spi_device, DM9051_NSR, NSR_CLR_STATUS);
    DM9051_write_reg(spi_device, DM9051_ISR, ISR_CLR_STATUS);

    /* edit */
#ifdef DM9051_FLOWCONTROL_EN
    DM9051_write_reg(spi_device, DM9051_FCR, FCR_FLOW_ENABLE); /* Flow Control */
#else
    DM9051_write_reg(spi_device, DM9051_FCR, 0x00); /* Flow Control */
#endif
    DM9051_write_reg(spi_device, DM9051_PPCR, PPCR_SETTING); /* Fully Pause Packet Count */
    DM9051_write_reg(spi_device, DM9051_NLEDCR, 0x81);       //set led model
    DM9051_write_reg(spi_device, DM9051_ATCR, 0x80);         //set TX auto_send
    DM9051_write_reg(spi_device, DM9051_BCASTCR, 0xC0);      //set rec broadcast packet

    DM9051_write_reg(spi_device, DM9051_RCR, RCR_DEFAULT);
}

static void dm9051_chip_reset(struct rt_spi_device *spi_device)
{
    //dbg_log("++\n");
    dm9051_soft_reset(spi_device);

    DM9051_write_reg(spi_device, DM9051_IMR, DM9051_IMR_SET);
}

#define DM9051_PHY              (0x40)    /* PHY address 0x01 */
static void phy_write(struct rt_spi_device *spi_device, uint16_t reg, uint16_t value)
{
    /* Fill the phyxcer register into REG_0C */
    DM9051_write_reg(spi_device, DM9051_EPAR, DM9051_PHY | reg);

    /* Fill the written data into REG_0D & REG_0E */
    DM9051_write_reg(spi_device, DM9051_EPDRL, (value & 0xff));
    DM9051_write_reg(spi_device, DM9051_EPDRH, ((value >> 8) & 0xff));
    DM9051_write_reg(spi_device, DM9051_EPCR, 0xa); /* Issue phyxcer write command */

    while (DM9051_read_reg(spi_device, DM9051_EPCR) & 0x1)
    {
        rt_thread_delay(1);
    }; //Wait complete

    DM9051_write_reg(spi_device, DM9051_EPCR, 0x0); /* Clear phyxcer write command */
}

static uint16_t phy_read(struct rt_spi_device *spi_device, uint32_t reg_oft)
{
    uint16_t value;

    /* Fill the phyxcer register into REG_0C */
    DM9051_write_reg(spi_device, DM9051_EPAR, DM9051_PHY | reg_oft);
    DM9051_write_reg(spi_device, DM9051_EPCR, 0xc); /* Issue phyxcer read command */

    while (DM9051_read_reg(spi_device, DM9051_EPCR) & 0x1)
    {
        rt_thread_delay(1);
    }; //Wait complete

    DM9051_write_reg(spi_device, DM9051_EPCR, 0x0); /* Clear phyxcer read command */
    value = (DM9051_read_reg(spi_device, DM9051_EPDRH) << 8) | DM9051_read_reg(spi_device, DM9051_EPDRL);

    return value;
}

static void phy_mode_set(struct rt_spi_device *spi_device)
{
    uint16_t phy_reg4 = 0x01e1, phy_reg0 = 0x1200;

    phy_write(spi_device, 20, 0x0200); /* Disable NWAY powersaver */

#ifdef DM9051_FLOWCONTROL_EN
    phy_write(spi_device, 4, phy_reg4 | 0x0400); /* Set PHY media mode */
#else
    phy_write(spi_device, 4, phy_reg4); /* Set PHY media mode */
#endif /* DM9051_FLOWCONTROL_EN */
    phy_write(spi_device, 0, phy_reg0); /* RE_START NWAY */
}

#if (LWIP_IPV4 && LWIP_IGMP) || (LWIP_IPV6 && LWIP_IPV6_MLD)

/* polynomial: 0xEDB88320L */
static uint32_t crc32_le(const uint8_t *data, size_t length)
{
    uint32_t crc = 0xffffffff;

    int i;
    while (length--)
    {
        crc ^= *data++;
        for (i = 0; i < 8; i++)
        {
            crc = (crc >> 1) ^ ((crc & 1) ? 0xEDB88320L : 0);
        }
    }
    return crc;
}

static void register_multicast_address(struct rt_dm9051_eth *eth, const uint8_t *mac, u8_t action)
{
    struct rt_spi_device *spi_device;
    spi_device = eth->spi_device;

    uint32_t crc;
    uint8_t hash;
    uint8_t hash_group;

    /* calculate crc32 value of mac address */
    crc = crc32_le(mac, 6);

    hash = crc & 0x3F;
    LOG_D("register_%s multicast_address crc: %08X hash: %02X\n",
          ((action) ? "add" : "del"), crc, hash);

    hash_group = hash / 8;

    if (hash > 31)
    {
        if (action) // NETIF_ADD_MAC_FILTER
        {
            eth->HashTableHigh |= 1 << (hash - 32);
        }
        else // NETIF_DEL_MAC_FILTER
        {
            eth->HashTableHigh &= ~(1 << (hash - 32));
        }
        DM9051_write_reg(spi_device, DM9051_MAR + hash_group, (eth->HashTableHigh >> (hash_group - 4) * 8) & 0xff);
    }
    else
    {
        if (action) // NETIF_ADD_MAC_FILTER
        {
            eth->HashTableLow |= 1 << hash;
        }
        else // NETIF_DEL_MAC_FILTER
        {
            eth->HashTableLow &= ~(1 << hash);
        }
        DM9051_write_reg(spi_device, DM9051_MAR + hash_group, (eth->HashTableLow >> (hash_group * 8)) & 0xff);
    }
    /*
    {
        LOG_I("HashTableHigh %08lx , HashTableLow %08lx ",
                eth->HashTableHigh , eth->HashTableLow );
        uint8_t mar[8];
        uint8_t i, oft;
        for (i = 0, oft = DM9051_MAR; i < sizeof(mar); i++, oft++)
        {
            mar[i] = DM9051_read_reg(spi_device, oft);
        }
        LOG_I("MAR: %02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X",
                mar[0], mar[1], mar[2], mar[3], mar[4], mar[5], mar[6], mar[7]);
    }
    */
}
#endif /* (LWIP_IPV4 && LWIP_IGMP) || (LWIP_IPV6 && LWIP_IPV6_MLD) */

#if LWIP_IPV4 && LWIP_IGMP
static err_t igmp_mac_filter( struct netif *netif, const ip4_addr_t *ip4_addr, u8_t action )
{
    uint8_t mac[6];
    const uint8_t *p = (const uint8_t *)ip4_addr;
    struct rt_dm9051_eth *eth = (struct rt_dm9051_eth *)netif->state;

    mac[0] = 0x01;
    mac[1] = 0x00;
    mac[2] = 0x5E;
    mac[3] = *(p+1) & 0x7F;
    mac[4] = *(p+2);
    mac[5] = *(p+3);

    register_multicast_address(eth, mac, action);

    LOG_D("%s %s %s ",
            __FUNCTION__, (action==NETIF_ADD_MAC_FILTER)?"add":"del", ip4addr_ntoa(ip4_addr));
    LOG_D("%02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    return 0;
}
#endif /* LWIP_IPV4 && LWIP_IGMP */

#if LWIP_IPV6 && LWIP_IPV6_MLD
static err_t mld_mac_filter( struct netif *netif, const ip6_addr_t *ip6_addr, u8_t action )
{
    uint8_t mac[6];
    const uint8_t *p = (const uint8_t *)&ip6_addr->addr[3];
    struct rt_dm9051_eth *eth = (struct rt_dm9051_eth *)netif->state;

    mac[0] = 0x33;
    mac[1] = 0x33;
    mac[2] = *(p+0);
    mac[3] = *(p+1);
    mac[4] = *(p+2);
    mac[5] = *(p+3);

    register_multicast_address(eth, mac, action);

    LOG_D("%s %s %s ",
             __FUNCTION__, (action==NETIF_ADD_MAC_FILTER)?"add":"del", ip6addr_ntoa(ip6_addr));
    LOG_D("%02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    return 0;
}
#endif /* LWIP_IPV6 && LWIP_IPV6_MLD */

/* initialize the interface */
static rt_err_t dm9051_init(rt_device_t dev)
{
    struct rt_dm9051_eth *eth;
    struct rt_spi_device *spi_device;
    int i, oft;

    eth = (struct rt_dm9051_eth *)dev;
    spi_device = eth->spi_device;

    dm9051_lock(dev);

    rt_pin_mode(eth->rst_pin, PIN_MODE_OUTPUT);
    rt_pin_write(eth->rst_pin, PIN_LOW);
    rt_thread_delay(2);
    rt_pin_write(eth->rst_pin, PIN_HIGH);
    rt_thread_delay(2);

    /* RESET device */
    DM9051_write_reg(spi_device, DM9051_NCR, DM9051_NCR_REG_RESET);
    rt_thread_delay(2);
    DM9051_write_reg(spi_device, DM9051_NCR, 0);

    DM9051_write_reg(spi_device, DM9051_GPCR, GPCR_GEP_CNTL);
    DM9051_write_reg(spi_device, DM9051_GPR, 0x00); //Power on PHY

    rt_thread_delay(100);

    /* Set PHY */
    phy_mode_set(spi_device);

    /* set mac address */
    for (i = 0, oft = DM9051_PAR; i < 6; i++, oft++)
    {
        DM9051_write_reg(spi_device, oft, eth->dev_addr[i]);
    }

    {
        uint8_t mac[6];
        for (i = 0, oft = DM9051_PAR; i < sizeof(mac); i++, oft++)
        {
            mac[i] = DM9051_read_reg(spi_device, oft);
        }
        LOG_I("MAC: %02X-%02X-%02X-%02X-%02X-%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }

    {
        for (i = 0, oft = DM9051_MAR; i < 8; i++, oft++)
        {
            DM9051_write_reg(spi_device, oft, 0x00);
        }
        LOG_I("Clean Multicast Address Hash Table");
#if (LWIP_IPV4 && LWIP_IGMP) || (LWIP_IPV6 && LWIP_IPV6_MLD)
        eth->HashTableHigh = 0;
        eth->HashTableLow = 0;
#endif
    }

    /* Activate DM9051 */
    dm9051_soft_reset(spi_device);

#ifdef DM9051_FLOWCONTROL_EN
    LOG_I("Enable Flow_Control Function");
#else
    LOG_I("Disable Flow_Control Function");
#endif

    rt_pin_irq_enable(eth->int_pin, PIN_IRQ_ENABLE);
    rt_timer_start(&eth->timer);

    dm9051_unlock(dev);

    return RT_EOK;
}

/* control the interface */
static rt_err_t dm9051_control(rt_device_t dev, int cmd, void *args)
{
    struct rt_dm9051_eth *eth;
    eth = (struct rt_dm9051_eth *)dev;

    switch (cmd)
    {
    case NIOCTL_GADDR:
        /* get mac address */
        if (args)
            rt_memcpy(args, eth->dev_addr, 6);
        else
            return -RT_ERROR;
        break;

    default:
        break;
    }

    return RT_EOK;
}

/* Open the ethernet interface */
static rt_err_t dm9051_open(rt_device_t dev, uint16_t oflag)
{
    LOG_D("[%s L%d]", __FUNCTION__, __LINE__);
    return RT_EOK;
}

/* Close the interface */
static rt_err_t dm9051_close(rt_device_t dev)
{
    LOG_D("[%s L%d]", __FUNCTION__, __LINE__);
    return RT_EOK;
}

/* Read */
static rt_size_t dm9051_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size)
{
    LOG_D("[%s L%d]", __FUNCTION__, __LINE__);
    rt_set_errno(-RT_ENOSYS);
    return RT_EOK;
}

/* Write */
static rt_size_t dm9051_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size)
{
    LOG_D("[%s L%d]", __FUNCTION__, __LINE__);
    rt_set_errno(-RT_ENOSYS);
    return 0;
}

/* ethernet device interface */
/* Transmit packet. */
static rt_err_t dm9051_tx(rt_device_t dev, struct pbuf *p)
{
    struct rt_dm9051_eth *eth;
    struct rt_spi_device *spi_device;
    void *tmp_buf = RT_NULL;

    eth = (struct rt_dm9051_eth *)dev;
    spi_device = eth->spi_device;

    dm9051_lock(dev);

    //LOG_D("[%s L%d]\n", __FUNCTION__, __LINE__);

#ifdef DM9051_TX_DUMP
    packet_dump(__FUNCTION__, p);
#endif /* DM9051_TX_DUMP */

    {
        rt_uint32_t retry = 0;
        uint8_t nsr_reg = 0;

        // wait for sending complete
        while (1)
        {
            nsr_reg = DM9051_read_reg(spi_device, DM9051_NSR) & (NSR_TX1END | NSR_TX2END);
            if(nsr_reg != 0)
            {
                break;
            }

            retry++;
            if (retry > 10)
            {
                LOG_E("wait for send complete timeout, retry=%d, abort!", retry);
                goto _exit;
            }

            rt_thread_delay(1);
        }

        if (retry > 2)
        {
            LOG_E("TX wait %d.", retry);
        }

        if ((NSR_TX1END | NSR_TX2END) == nsr_reg)
        {
            DM9051_write_reg(spi_device, DM9051_MPCR, 0x02); //reset tx point
        }
    }

    if (p->tot_len != p->len)
    {
        LOG_D("[%s L%d], tot_len:len ==> %d:%d", __FUNCTION__, __LINE__, p->tot_len, p->len);

        tmp_buf = (void *)rt_malloc(p->tot_len);
        if (!tmp_buf)
        {
            LOG_W("[%s L%d], no memory for pbuf, len=%d.", __FUNCTION__, __LINE__, p->tot_len);
            goto _exit;
        }

        pbuf_copy_partial(p, tmp_buf, p->tot_len, 0);
    }

    //Write data to FIFO
    DM9051_write_reg(spi_device, DM9051_TXPLL, p->tot_len & 0xff);
    DM9051_write_reg(spi_device, DM9051_TXPLH, (p->tot_len >> 8) & 0xff);

    if (tmp_buf)
    {
        DM9051_write_mem(spi_device, tmp_buf, p->tot_len);
    }
    else
    {
        DM9051_write_mem(spi_device, p->payload, p->tot_len);
    }

#ifdef DM9051_PACKET_CNT_EN
    eth->tx_count++;
#endif /* DM9051_PACKET_CNT_EN */

_exit:
    if (tmp_buf)
    {
        rt_free(tmp_buf);
    }

    dm9051_unlock(dev);

    return RT_EOK;
}

/* recv packet. */
static struct pbuf *dm9051_rx(rt_device_t dev)
{
    struct rt_dm9051_eth *eth;
    struct rt_spi_device *spi_device;

    uint8_t isr_reg, rxbyte;
    uint8_t nsr_reg;
    struct pbuf *p = RT_NULL;

    eth = (struct rt_dm9051_eth *)dev;
    spi_device = eth->spi_device;

    LOG_D("[%s L%d]", __FUNCTION__, __LINE__);
    rt_timer_stop(&eth->timer);

    dm9051_lock(dev);

    DM9051_write_reg(spi_device, DM9051_IMR, DM9051_IMR_OFF); // Disable all interrupts
    isr_reg = DM9051_read_reg(spi_device, DM9051_ISR);
    DM9051_write_reg(spi_device, DM9051_ISR, (isr_reg & ISR_CLR_STATUS));  // Clear ISR status
    LOG_D("isr_reg=0x%x", isr_reg);

    /*********** link status check*************/
    if (isr_reg & ISR_LNKCHGS)
    {
        nsr_reg = DM9051_read_reg(spi_device, DM9051_NSR);
        if (nsr_reg & NSR_LINKST)
        {
#ifdef DM9051_WAIT_NWAY_LINK_EN
            uint8_t lnk_status;
            uint8_t ncr_reg;

            rt_thread_delay(rt_tick_from_millisecond(5000));

            ncr_reg = DM9051_read_reg(spi_device, DM9051_NCR) & NCR_FDX;
            nsr_reg = DM9051_read_reg(spi_device, DM9051_NSR) & (NSR_SPEED | NSR_LINKST);

            lnk_status = nsr_reg | ncr_reg;
            LOG_D("[%s L%d] DM9051_NSR=0x%08X", __FUNCTION__, __LINE__, lnk_status);
            switch (lnk_status)
            {
            case (NSR_LINKST):
                LOG_I("link up phy 100M hal duplex!", __FUNCTION__, __LINE__);
                break;
            case (NSR_LINKST | NCR_FDX):
                LOG_I("link up phy 100M full duplex!", __FUNCTION__, __LINE__);
                break;
            case (NSR_SPEED | NSR_LINKST):
                LOG_I("link up phy 10M hal duplex!", __FUNCTION__, __LINE__);
                break;
            case (NSR_SPEED | NSR_LINKST | NCR_FDX):
                LOG_I("link up phy 10M full duplex!", __FUNCTION__, __LINE__);
                break;

            default:
                break;
            }
#else
            LOG_I("link up!", __FUNCTION__, __LINE__);
#endif /* DM9051_WAIT_NWAY_LINK_EN */
            eth_device_linkchange(&eth->parent, RT_TRUE);
        }
        else
        {
            LOG_W("link down!\n", __FUNCTION__, __LINE__);
            eth_device_linkchange(&eth->parent, RT_FALSE);
        }
    } /* link status check */

    // Receive Overflow Counter Overflow
    if (isr_reg & ISR_ROOS)
    {
        LOG_E("dm9051_chip_reset Receive Overflow Counter Overflow");
        dm9051_chip_reset(spi_device);
        goto _exit;
    }

    // Receive Overflow
    if (isr_reg & ISR_ROS)
    {
        LOG_W("Receive_FIFO Overflow");
    }

    // transmit
    if (isr_reg & ISR_PTS)
    {
        LOG_D("ISR_PTS");
    }

    if (isr_reg & ISR_PRS)
    {
        uint16_t rx_status, rx_len;
        uint8_t ReceiveData[4];

        /* Check packet ready or not                                                                              */
        nsr_reg = DM9051_read_reg(spi_device, DM9051_NSR) & NSR_RXRDY;
        if (nsr_reg)
        {
            rxbyte = DM9051_read_reg(spi_device, DM9051_MRCMDX);
            rxbyte = DM9051_read_reg(spi_device, DM9051_MRCMDX1);
            LOG_D("MRCMDX = %02X.", rxbyte);

            if (rxbyte != 0x01)
            {
                LOG_E("NSR %02X, RCMDX %02X: rx error, dm9051_chip_rx_fifo_reset", nsr_reg, rxbyte);
                DM9051_write_reg(spi_device, DM9051_MPCR, 0x01); //reset rx point
                DM9051_write_reg(spi_device, DM9051_ISR, ISR_CLR_RX_STATUS);

                goto _exit;
            }
        }
        else
        {
            DM9051_write_reg(spi_device, DM9051_ISR, ISR_CLR_RX_STATUS);

            goto _exit;
        }

        DM9051_read_mem(spi_device, ReceiveData, 4);

        rx_status = ReceiveData[0] + (ReceiveData[1] << 8);
        rx_len = ReceiveData[2] + (ReceiveData[3] << 8);

        if ((rx_status & 0x3900) || rx_len < 12)
        {
            LOG_W("rx_status = %04X. rx_len = %04X.", rx_status, rx_len);
        }
        else
        {
            LOG_D("rx_status = %04X. rx_len = %04X.", rx_status, rx_len);
        }

        /* allocate buffer           */
        p = pbuf_alloc(PBUF_LINK, rx_len, PBUF_RAM);
        if (p != NULL)
        {
#ifdef DM9051_PACKET_CNT_EN
            eth->rx_count++;
#endif /* DM9051_PACKET_CNT_EN */
            DM9051_read_mem(spi_device, (u8_t *)p->payload, rx_len);
        }
        else
        {
            LOG_W("DM9051 rx: no pbuf.");

#if (DM9051_NO_PBUF_LEVEL >= 2)
            //keep data in dm9051 fifo
            uint16_t wRx_point;

            wRx_point = (DM9051_read_reg(spi_device, DM9051_MRRH) << 8);
            wRx_point |= DM9051_read_reg(spi_device, DM9051_MRRL);

            wRx_point -= 04;
            if (wRx_point < 0x0c00)
            {
                wRx_point += 0x3400;
            }

            DM9051_write_reg(spi_device, DM9051_MRRL, wRx_point & 0xff);
            DM9051_write_reg(spi_device, DM9051_MRRH, (wRx_point >> 8) & 0xff);
#elif (DM9051_NO_PBUF_LEVEL == 1)
            // jump one packet
            uint16_t wRx_point;

            wRx_point = (DM9051_read_reg(spi_device, DM9051_MRRH) << 8);
            wRx_point |= DM9051_read_reg(spi_device, DM9051_MRRL);

            wRx_point += rx_len;
            if (wRx_point > 0x3fff)
            {
                wRx_point -= 0x3400;
            }

            DM9051_write_reg(spi_device, DM9051_MRRL, wRx_point & 0xff);
            DM9051_write_reg(spi_device, DM9051_MRRH, (wRx_point >> 8) & 0xff);
#else
            //clean rx fifo data
            DM9051_write_reg(spi_device, DM9051_MPCR, 0x01); //reset rx point
            DM9051_write_reg(spi_device, DM9051_ISR, ISR_CLR_RX_STATUS);
#endif
        }
    } /* isr_reg & ISR_PRS */

#ifdef DM9051_RX_DUMP
    if(p)
        packet_dump(__FUNCTION__, p);
#endif /* DM9051_RX_DUMP */

_exit:
    DM9051_write_reg(spi_device, DM9051_IMR, DM9051_IMR_SET); // Re-enable interrupt mask

    dm9051_unlock(dev);

    if (!p)
    {
        LOG_D("re-enable DM9051 INT PIN IRQ \r\n");
        rt_pin_irq_enable(eth->int_pin, PIN_IRQ_ENABLE);
        rt_timer_start(&eth->timer);
    }

    return p;
}

#ifdef RT_USING_DEVICE_OPS
const static struct rt_device_ops dm9051_ops =
{
    dm9051_init,
    dm9051_open,
    dm9051_close,
    dm9051_read,
    dm9051_write,
    dm9051_control
};
#endif /* RT_USING_DEVICE_OPS */

static struct rt_dm9051_eth *dm9051_monitor;

static void dm9051_isr(void *args)
{
    struct rt_dm9051_eth *eth = (struct rt_dm9051_eth *)args;

    eth->irq_count++;

    rt_pin_irq_enable(eth->int_pin, PIN_IRQ_DISABLE);
    LOG_D("[%s L%d] \n", __FUNCTION__, __LINE__);

    eth_device_ready(&eth->parent);
}

static void dm9051_timeout(void* parameter)
{
    struct rt_dm9051_eth *eth = (struct rt_dm9051_eth *)parameter;

    LOG_D("[%s L%d] eth_device_ready", __FUNCTION__, __LINE__);
    eth_device_ready(&eth->parent);
}

int dm9051_probe(const char *spi_dev_name, const char *device_name, int rst_pin, int int_pin)
{
    struct rt_dm9051_eth *eth;
    uint32_t device_id;

    struct rt_spi_device *spi_device;

    spi_device = (struct rt_spi_device *)rt_device_find(spi_dev_name);
    if (spi_device == RT_NULL)
    {
        LOG_E("[%s:%d] spi device %s not found!.", __FUNCTION__, __LINE__, spi_dev_name);
        return -RT_ENOSYS;
    }

    /* config spi */
    {
        struct rt_spi_configuration cfg;
        cfg.data_width = 8;
        cfg.mode = RT_SPI_MODE_0 | RT_SPI_MSB; /* SPI Compatible: Mode 0 and Mode 3 */
        cfg.max_hz = DM9051_SPI_MAX_HZ;               /* defaut=20M, max 50M  */
        rt_spi_configure(spi_device, &cfg);
    }

    device_id  = DM9051_read_reg(spi_device, DM9051_VIDL);
    device_id |= DM9051_read_reg(spi_device, DM9051_VIDH) << 8;
    device_id |= DM9051_read_reg(spi_device, DM9051_PIDL) << 16;
    device_id |= DM9051_read_reg(spi_device, DM9051_PIDH) << 24;
    LOG_I("[%s L%d] device_id: %08X", __FUNCTION__, __LINE__, device_id);

    if(device_id != DM9051_ID)
    {
        return -1;
    }

    device_id  = DM9051_read_reg(spi_device, DM9051_CHIPR);
    LOG_I("[%s L%d] CHIP Revision: %02X", __FUNCTION__, __LINE__, device_id);

    eth = rt_calloc(1, sizeof(struct rt_dm9051_eth));
    if(!eth)
    {
        return -1;
    }

    eth->spi_device = spi_device;
    eth->rst_pin = rst_pin;
    eth->int_pin = int_pin;

    /* OUI 00-60-6E Davicom Semiconductor, Inc. */
    eth->dev_addr[0] = 0x00;
    eth->dev_addr[1] = 0x60;
    eth->dev_addr[2] = 0x6E;
    eth->dev_addr[3] = 12;
    eth->dev_addr[4] = 34;
    eth->dev_addr[5] = 56;

    rt_pin_mode(eth->int_pin, PIN_MODE_INPUT_PULLDOWN);
    device_id = rt_pin_attach_irq(eth->int_pin, PIN_IRQ_MODE_RISING, dm9051_isr, eth); /* default: push-pull, high active. PIN_IRQ_MODE_HIGH_LEVEL or PIN_IRQ_MODE_RISING */
    LOG_D("[%s L%d] rt_pin_attach_irq #%d res:%d \n", __FUNCTION__, __LINE__, eth->int_pin, device_id);

    /* init rt-thread device struct */
    eth->parent.parent.type = RT_Device_Class_NetIf;
#ifdef RT_USING_DEVICE_OPS
    eth->parent.parent.ops = &dm9051_ops;
#else
    eth->parent.parent.init    = dm9051_init;
    eth->parent.parent.open    = dm9051_open;
    eth->parent.parent.close   = dm9051_close;
    eth->parent.parent.read    = dm9051_read;
    eth->parent.parent.write   = dm9051_write;
    eth->parent.parent.control = dm9051_control;
#endif /* RT_USING_DEVICE_OPS */

    /* init rt-thread ethernet device struct */
    eth->parent.eth_rx  = dm9051_rx;
    eth->parent.eth_tx  = dm9051_tx;

    rt_mutex_init(&eth->lock, "dm9051", RT_IPC_FLAG_FIFO);
    rt_timer_init(&eth->timer, "dm9051", dm9051_timeout, eth, RT_TICK_PER_SECOND/2, RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);

    dm9051_monitor = eth;

    eth_device_init(&eth->parent, "e0");

#if LWIP_IPV4 && LWIP_IGMP
    netif_set_igmp_mac_filter(eth->parent.netif, igmp_mac_filter);
#endif

#if LWIP_IPV6 && LWIP_IPV6_MLD
    netif_set_mld_mac_filter(eth->parent.netif, mld_mac_filter);
#endif

    return 0;
}

static int dm9051_dump(void)
{
    uint32_t value;
    uint32_t pos = 0;
    struct rt_spi_device *spi_device = dm9051_monitor->spi_device;

    rt_kprintf("[%s L%d], irq count:%d.\n", __FUNCTION__, __LINE__, dm9051_monitor->irq_count);
    dm9051_monitor->irq_count = 0;

#ifdef DM9051_PACKET_CNT_EN
    rt_kprintf("[%s L%d], tx_count:%d.\n", __FUNCTION__, __LINE__, dm9051_monitor->tx_count);
    rt_kprintf("[%s L%d], rx_count:%d.\n", __FUNCTION__, __LINE__, dm9051_monitor->rx_count);
    dm9051_monitor->tx_count = 0;
    dm9051_monitor->rx_count = 0;
#endif /* DM9051_PACKET_CNT_EN */

    pos=0;
    value = DM9051_read_reg(spi_device, pos);
    rt_kprintf("#%02X 0x%02X NCR\n", pos, value);

    pos=1;
    value = DM9051_read_reg(spi_device, pos);
    rt_kprintf("#%02X 0x%02X NSR\n", pos, value);

    pos=2;
    value = DM9051_read_reg(spi_device, pos);
    rt_kprintf("#%02X 0x%02X TCR\n", pos, value);

    pos=3;
    value = DM9051_read_reg(spi_device, pos);
    rt_kprintf("#%02X 0x%02X TSR1\n", pos, value);

    pos=4;
    value = DM9051_read_reg(spi_device, pos);
    rt_kprintf("#%02X 0x%02X TSR2\n", pos, value);

    pos=5;
    value = DM9051_read_reg(spi_device, pos);
    rt_kprintf("#%02X 0x%02X RCR\n", pos, value);

    pos=6;
    value = DM9051_read_reg(spi_device, pos);
    rt_kprintf("#%02X 0x%02X RSR\n", pos, value);

    pos=7;
    value = DM9051_read_reg(spi_device, pos);
    rt_kprintf("#%02X 0x%02X ROCR, recv overflow count\n", pos, value);

    pos=0x39;
    value = DM9051_read_reg(spi_device, pos);
    rt_kprintf("#%02X 0x%02X INTCR\n", pos, value);

    pos=0x3E;
    value = DM9051_read_reg(spi_device, pos);
    rt_kprintf("#%02X 0x%02X EEE_IN, 802.3az enter count\n", pos, value);

    pos=0x3F;
    value = DM9051_read_reg(spi_device, pos);
    rt_kprintf("#%02X 0x%02X EEE_OUT, 802.3az leave count\n", pos, value);

    pos=0x59;
    value = DM9051_read_reg(spi_device, pos);
    rt_kprintf("#%02X 0x%02X memory control. ", pos, value);
    rt_kprintf("%s \n", 
    (value&0x01)?"By 5AH":"3K for TX, 13K for RX." );

    if(value&0x01)
    {
        pos = 0x5A;
        value = DM9051_read_reg(spi_device, pos);
        rt_kprintf("#%02X 0x%02X TRAM_SIZE.\n", pos, value);
    }

    pos=0x7E;
    value = DM9051_read_reg(spi_device, pos);
    rt_kprintf("#%02X 0x%02X ISR, interrupt status\n", pos, value);

    pos=0x7F;
    value = DM9051_read_reg(spi_device, pos);
    rt_kprintf("#%02X 0x%02X IMR, interrupt mask\n", pos, value);


    rt_kprintf("\n" "DM9051 PHY register dump.\n");

    pos = DM9051_PHY_REG_BMCR;
    value = phy_read(spi_device, pos);
    rt_kprintf("#%02X 0x%04X BMCR\n", pos, value);

    pos = DM9051_PHY_REG_BMSR;
    value = phy_read(spi_device, pos);
    value = phy_read(spi_device, pos); //update_value
    rt_kprintf("#%02X 0x%04X BMSR\n", pos, value);

    /* ID1=0181, ID2=B8A0, OUI=0x0000C0DC, Vendor Model:0x0A, Model Revision:0x00. */
    value = phy_read(spi_device, DM9051_PHY_REG_PHYID1);
    value = (value << 16) | phy_read(spi_device, DM9051_PHY_REG_PHYID2);
    rt_kprintf("#%02X 0x%08X PHYID, OUI=0x%08X, Vendor Model:0x%02X, Model Revision:0x%02X.\n", pos, value, value>>9, (value>>4)&0x3F, value&0x0F);

    pos = DM9051_PHY_REG_ANAR;
    value = phy_read(spi_device, pos);
    rt_kprintf("#%02X 0x%04X ANAR\n", pos, value);

    pos = DM9051_PHY_REG_ANLPAR;
    value = phy_read(spi_device, pos);
    rt_kprintf("#%02X 0x%04X ANLPAR\n", pos, value);

    pos = DM9051_PHY_REG_ANER;
    value = phy_read(spi_device, pos);
    rt_kprintf("#%02X 0x%04X ANER\n", pos, value);

    pos = DM9051_PHY_REG_DSCR;
    value = phy_read(spi_device, pos);
    rt_kprintf("#%02X 0x%04X DSCR\n", pos, value);

    pos = DM9051_PHY_REG_DSCSR;
    value = phy_read(spi_device, pos);
    rt_kprintf("#%02X 0x%04X DSCSR\n", pos, value);

    pos = DM9051_PHY_REG_10BTCSR;
    value = phy_read(spi_device, pos);
    rt_kprintf("#%02X 0x%04X 10BTCSR\n", pos, value);

    pos = DM9051_PHY_REG_PWDOR;
    value = phy_read(spi_device, pos);
    rt_kprintf("#%02X 0x%04X PWDOR\n", pos, value);

    pos = DM9051_PHY_REG_SCR;
    value = phy_read(spi_device, pos);
    rt_kprintf("#%02X 0x%04X SCR\n", pos, value);

    pos = DM9051_PHY_REG_PSCR;
    value = phy_read(spi_device, pos);
    rt_kprintf("#%02X 0x%04X PSCR\n", pos, value);

    return 0;
}
MSH_CMD_EXPORT(dm9051_dump, dump dm9051 register);
