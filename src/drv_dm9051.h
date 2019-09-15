#ifndef _DM9051_H_INC_
#define _DM9051_H_INC_

#define DM9051_NCR             	(0x00)
#define DM9051_NSR             	(0x01)
#define DM9051_TCR              (0x02)
#define DM9051_RCR              (0x05)
#define DM9051_BPTR             (0x08)
#define DM9051_FCTR             (0x09)
#define DM9051_FCR              (0x0A)
#define DM9051_EPCR             (0x0B)
#define DM9051_EPAR             (0x0C)
#define DM9051_EPDRL            (0x0D)
#define DM9051_EPDRH            (0x0E)
#define DM9051_PAR              (0x10)
#define DM9051_GPCR	       		(0x1E)
#define DM9051_GPR              (0x1f)
#define DM9051_TRPAL            (0x22)
#define DM9051_TRPAH            (0x23)
#define DM9051_RWPAL            (0x24)
#define DM9051_RWPAH            (0x25)

#define DM9051_VIDL             (0x28)
#define DM9051_VIDH             (0x29)
#define DM9051_PIDL             (0x2A)
#define DM9051_PIDH             (0x2B)

#define DM9051_CHIPR            (0x2C)
#define DM9051_TCR2             (0x2D)
#define DM9051_OTCR             (0x2E)
#define DM9051_SMCR             (0x2F)

#define DM9051_INTCR			0x39
#define DM9051_PPCR				0x3D

#define DM9051_INTR			    (0x39)
#define DM9051_MPCR	       		(0x55)

#define DM9051_MRRL             0x74 //0xF4
#define DM9051_MRRH             0x75 //0xF5
#define DM9051_MWRL             0x7A //0xFA
#define DM9051_MWRH             0x7B //0xFB
#define DM9051_TXPLL            0x7C //0xFC
#define DM9051_TXPLH            0x7D //0xFD
#define DM9051_ISR             	0x7E //0xFE
#define DM9051_IMR             	0x7F //0xFF

#define DM9051_MRCMDX           (0x70)
#define DM_SPI_MRCMDX			(0x70)
#define DM_SPI_MRCMD			(0x72)
#define DM_SPI_MWCMD			(0x78)

#define DM_SPI_RD				(0x00)
#define DM_SPI_WR				(0x80)

/* DM9051 PHY register list */
#define DM9051_PHY_REG_BMCR     (0x00) /* Basic Mode Control Register */
#define DM9051_PHY_REG_BMSR     (0x01) /* Basic Mode Status Register */
#define DM9051_PHY_REG_PHYID1   (0x02) /* PHY ID Identifier Register #1 */
#define DM9051_PHY_REG_PHYID2   (0x03) /* PHY ID Identifier Register #2 */
#define DM9051_PHY_REG_ANAR     (0x04) /* Auto-Negotiation Advertisement Register */
#define DM9051_PHY_REG_ANLPAR   (0x05) /* Auto-Negotiation Link Partner Ability Register */
#define DM9051_PHY_REG_ANER     (0x06) /* Auto-Negotiation Expansion Register */
#define DM9051_PHY_REG_DSCR     (0x10) /* DAVICOM Specified Configuration Register  */
#define DM9051_PHY_REG_DSCSR    (0x11) /* DAVICOM Specified Configuration and Status Register  */
#define DM9051_PHY_REG_10BTCSR  (0x12) /* 10BASE-T Configuration/Status */
#define DM9051_PHY_REG_PWDOR    (0x13) /* Power Down Control Register */
#define DM9051_PHY_REG_SCR      (0x14) /* Specified Config Register */
#define DM9051_PHY_REG_PSCR     (0x1D) /* Power Saving Control Register */

/********* register define *********/
#define DM9051_NCR_REG_RESET    (0x01)
#define NCR_DEFAULT		        (0x00)						// Disable Wakeup

//0x01
#define NSR_SPEED           (1 << 7)
#define NSR_LINKST          (1 << 6)
#define NSR_WAKEST          (1 << 5)
#define NSR_TX2END          (1 << 3)
#define NSR_TX1END          (1 << 2)
#define NSR_RXOV            (1 << 1)
#define NSR_CLR_STATUS		(NSR_WAKEST | NSR_TX2END | NSR_TX1END)

/* 0x02 */
#define TCR_TJDIS           (1 << 6)
#define TCR_EXCECM          (1 << 5)
#define TCR_PAD_DIS2        (1 << 4)
#define TCR_CRC_DIS2        (1 << 3)
#define TCR_PAD_DIS1        (1 << 2)
#define TCR_CRC_DIS1        (1 << 1)
#define TCR_TXREQ           (1 << 0)		//Start TX
#define TCR_DEFAULT		    (0x00)

//0x05
#define RCR_WTDIS           (1 << 6)
#define RCR_DIS_LONG        (1 << 5)
#define RCR_DIS_CRC         (1 << 4)
#define RCR_ALL	            (1 << 3)
#define RCR_RUNT            (1 << 2)
#define RCR_PRMSC           (1 << 1)
#define RCR_RXEN            (1 << 0)
#define RCR_RX_DISABLE      (RCR_DIS_LONG | RCR_DIS_CRC) // #define RCR_RX_DISABLE 0x30
#define RCR_DEFAULT		    (RCR_DIS_LONG | RCR_DIS_CRC | RCR_ALL | RCR_RUNT | RCR_RXEN)

#define BPTR_DEFAULT	    (0x3f)
#define FCTR_DEAFULT	    (0x38)
#define FCR_DEFAULT		    (0xFF)
#define SMCR_DEFAULT	    (0x00)

//0x0A
#define FCR_FLOW_ENABLE		 0x29
//0x1E
#define GPCR_GEP_CNTL       (1<<0)
//0x39
#define INTCR_POL       	(1<<0)
//0x3D
//#define PPCR_SETTING		 0x00 (Trouble in the way)
//#define PPCR_SETTING		 0x01 (default)
//#define PPCR_SETTING		 0x02 (TO BE TRY ONCE LATER)
//#define PPCR_SETTING		 0x08 (Using now, To work to)
#define PPCR_SETTING		 0x08
//0x55
#define MPCR_RSTTX          (1<<1)
#define MPCR_RSTRX          (1<<0)
//0xFE
#define ISR_LNKCHGS         (1<<5)
#define ISR_ROOS            (1<<3)
#define ISR_ROS             (1<<2)
#define ISR_PTS             (1<<1)
#define ISR_PRS             (1<<0)
#define ISR_CLR_STATUS      (ISR_LNKCHGS | ISR_ROOS | ISR_ROS | ISR_PTS | ISR_PRS)
//0xFF
#define IMR_PAR             (1<<7)
#define IMR_LNKCHGI         (1<<5)
#define IMR_ROOI            (1<<3)
#define IMR_ROI             (1<<2)
#define IMR_PTM             (1<<1)
#define IMR_PRM             (1<<0)
//Const
#define DM9051_PKT_RDY		0x01	/* Packet ready to receive */
#define DM9051_PKT_MAX		1536	/* Received packet max size */

#define DM9051_REG_RESET     (0x01)
#define DM9051_IMR_OFF       (0x80)
#define DM9051_TCR2_SET      (0x90)	//one packet
#define DM9051_RCR_SET       (0x31)
#define DM9051_BPTR_SET      (0x37)
#define DM9051_FCTR_SET      (0x38)
#define DM9051_FCR_SET       (0x28)
#define DM9051_TCR_SET       (0x01)

/*
 * dm9000 Ethernet
 */
//#define DM9000_NSR             0x01
#define DM9000_TCR             0x02
#define DM9000_RSR             0x06
#define DM9000_BPTR            0x08
#define DM9000_EPCR            0x0B
#define DM9000_EPAR            0x0C
#define DM9000_EPDRL           0x0D
#define DM9000_EPDRH           0x0E
#define DM9000_MAR             0x16
#define DM9000_GPR             0x1F
#define DM9000_SMCR            0x2F

//0x00
#define NCR_WAKEEN          (1<<6)
#define NCR_FDX             (1<<3)
#define NCR_RST	            (1<<0)

//0x06
#define RSR_RF              (1<<7)
#define RSR_MF              (1<<6)
#define RSR_LCS             (1<<5)
#define RSR_RWTO            (1<<4)
#define RSR_PLE             (1<<3)
#define RSR_AE              (1<<2)
#define RSR_CE              (1<<1)
#define RSR_FOE             (1<<0)
//0x0B
#define EPCR_WEP			(1<<4) //=0x10
#define EPCR_EPOS           (1<<3)
#define EPCR_ERPRR          (1<<2)
#define EPCR_ERPRW          (1<<1)
#define EPCR_ERRE           (1<<0)

#define DM9051_ID           (0x90510A46)    /* DM9051A ID */

#endif /* _DM9051_H_INC_ */
