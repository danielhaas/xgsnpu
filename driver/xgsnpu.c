// SPDX-License-Identifier: BSD-2-Clause
/*
 * xgsnpu - Linux host driver for the Marvell CN913x NPU (PCI 11ab:7080) that owns
 * every front port of a Sophos XGS 126/136, talking to the NPU's STOCK firmware.
 *
 * This is a Linux port of the FreeBSD `npuep` driver from
 * https://github.com/AbdelmonemAwad/os-xgs-npu (BSD-2-Clause), which brought all
 * fourteen front ports of an XGS 136 up under OPNsense. Protocol constants and the
 * bring-up order are taken from there; see that project's docs/ for provenance.
 *
 * Facilities published by the NPU (barmap at the top of BAR2):
 *   ctrl   - handshake + heartbeat, host-to-target doorbells (BAR4)
 *   giu    - the datapath: one queue set carrying all 14 ports, 66-byte prefix
 *   nwa    - NetAgent mailbox: admin up/down, MAC, link state per port
 *   rpc    - control-message ring: fills the fastpath's LIF / pport tables
 *   mvmgmt - host<->NPU management NIC (not used here)
 *
 * The NPU must have been released from reset (MCP2210 pulse) and the endpoint's BARs
 * restored (PCI remove + rescan) before this module is loaded. Do not autoload it.
 */

#include <linux/module.h>
#include <linux/pci.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/kthread.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/io-64-nonatomic-lo-hi.h>
#include <linux/unaligned.h>
#include <linux/dmi.h>
#include <linux/crc32.h>

#define DRV_NAME "xgsnpu"

static unsigned int stages = 0xf;
module_param(stages, uint, 0444);
MODULE_PARM_DESC(stages, "bring-up stages: 1=handshake 2=giu datapath 4=netagent 8=rpc tables");

static char *mac_src = "factory";
module_param(mac_src, charp, 0444);
MODULE_PARM_DESC(mac_src, "port MAC addresses: factory (Sophos-assigned, read from the NPU) or derived (02:<crc32 of DMI serial>:00:<port>)");

static unsigned int poll_us = 100;
module_param(poll_us, uint, 0644);
MODULE_PARM_DESC(poll_us, "datapath poll interval when idle, microseconds");

/* ------------------------------------------------------------------------------------------ */
/* barmap / ctrl facility (barmap.h, facility_conf.h)                                         */

#define NPU_BARMAP_COOKIE	0xD0FAC10DU
#define NPU_BARMAP_VERSION	5U
#define NPU_BARMAP_TAIL		0x104000U	/* window start, from the end of BAR2 */
#define NPU_BARMAP_WINDOW_LEN	0x103000U
#define NPU_BARMAP_STRUCT_OFF	0x102000U

#define ARMADA_FACILITY_COOKIE	0xAFACAFACU
#define CTRL_HANDSHAKE		0x04
#define  CTRL_TRGT_INIT		BIT(0)
#define  CTRL_HOST_INIT		BIT(1)
#define  CTRL_TRGT_H2T_DBELL	BIT(2)
#define  CTRL_HOST_ALIVE	BIT(3)
#define CTRL_H2T_DBELL_CNT	0x08
#define CTRL_H2T_DBELL_MSG	0x10
#define CTRL_DBELL_MSG_SIZE	16
#define CTRL_DBELL_MAX		16

enum { FAC_CTRL = 0, FAC_MGMT = 1, FAC_NWA = 2, FAC_RPC = 3, FAC_GIU = 4, FAC_COUNT = 5 };
static const char * const fac_name[FAC_COUNT] = { "ctrl", "mvmgmt", "nwa", "rpc", "giu" };
/* t2h doorbells per facility, flat MSI-X numbering in facility order */
#define NPU_TOTAL_DBELLS	5

#define NPU_DMA_BITS		36

/* ------------------------------------------------------------------------------------------ */
/* GIU (giu_nic_hw.h, via npugiu.h)                                                            */

#define AGNIC_CFG_STATUS		0x00
#define  AGNIC_STATUS_DEV_READY		0x1
#define  AGNIC_STATUS_HOST_MGMT_READY	0x2
#define  AGNIC_STATUS_DEV_MGMT_READY	0x4
#define AGNIC_CFG_MAC_ADDR		0x04
#define AGNIC_CFG_CMD_Q			0x10
#define AGNIC_CFG_NOTIF_Q		0x28
#define AGNIC_CFG_DEV_USE_SIZE		0x58
#define AGNIC_CFG_SIZE			0x400

#define AGNIC_QI_ADDR			0x00
#define AGNIC_QI_PROD_OFFS		0x08
#define AGNIC_QI_CONS_OFFS		0x0c
#define AGNIC_QI_LEN			0x10
#define AGNIC_QI_RES			0x14

#define AGNIC_CMD_IDX			0x00
#define AGNIC_CMD_APP_CODE		0x02
#define AGNIC_CMD_CODE			0x04
#define AGNIC_CMD_CLIENT_ID		0x05
#define AGNIC_CMD_CLIENT_TYPE		0x06
#define AGNIC_CMD_FLAGS			0x07
#define AGNIC_CMD_DATA			0x08
#define AGNIC_CMD_DESC_SIZE		0x40
#define AGNIC_MGMT_DATA_LEN		56

#define AGNIC_CMD_ID_ILLEGAL		0x0000
#define AGNIC_CMD_ID_NOTIFICATION	0xFFFF
#define AGNIC_AC_PF_MANAGER		0x2
#define AGNIC_CDT_PF			1
#define AGNIC_F_NO_RESP_SHIFT		5
#define AGNIC_F_BUF_POS_SHIFT		6
#define AGNIC_F_BUF_POS_MASK		0x3
#define  AGNIC_BUF_POS_SINGLE		0
#define  AGNIC_BUF_POS_LAST		2

#define AGNIC_CC_PF_INIT		0x01
#define AGNIC_CC_PF_INIT_DONE		0x02
#define AGNIC_CC_PF_EGRESS_TC_ADD	0x03
#define AGNIC_CC_PF_EGRESS_DATA_Q_ADD	0x04
#define AGNIC_CC_PF_INGRESS_TC_ADD	0x05
#define AGNIC_CC_PF_INGRESS_DATA_Q_ADD	0x06
#define AGNIC_CC_PF_ENABLE		0x07
#define AGNIC_CC_PF_DISABLE		0x08
#define AGNIC_CC_PF_MGMT_ECHO		0x09
#define AGNIC_CC_PF_LINK_STATUS		0x0a
#define AGNIC_CC_PF_CLOSE		0x0c
#define AGNIC_CC_PF_MAC_ADDR		0x0d
#define AGNIC_CC_PF_PROMISC		0x0e
#define AGNIC_CC_PF_MC_PROMISC		0x0f
#define AGNIC_CC_PF_MTU			0x10
#define AGNIC_CC_GET_CAPABILITIES	0x1e

#define AGNIC_NC_PF_LINK_CHANGE		0x1
#define AGNIC_NC_PF_KEEP_ALIVE		0x2

#define AGNIC_R_STATUS			0x00
#define AGNIC_R_CAP_FLAGS		0x01
#define AGNIC_R_CAP_MAX_BUF_SIZE	0x05
#define AGNIC_R_CAP_EGRESS_DMA		0x09
#define AGNIC_R_LINK_STATUS		0x01

#define AGNIC_ES_STRICT_SCHED		0x1
#define AGNIC_ING_HASH_NONE		0x0
#define AGNIC_ING_HASH_2_TUPLE		0x1

#define AGNIC_TXD_FLAGS			0x00
#define AGNIC_TXD_PKT_OFFSET		0x04
#define AGNIC_TXD_BYTE_CNT		0x06
#define AGNIC_TXD_BUFFER_ADDR		0x10
#define AGNIC_TXD_COOKIE		0x18
#define AGNIC_TXD_SIZE			0x20
#define AGNIC_TXD_F_GEN_L4_CSUM_NOT	BIT(14)
#define AGNIC_TXD_F_GEN_IPV4_CSUM_DIS	BIT(15)
#define AGNIC_TXD_F_MD_MODE		BIT(22)

#define AGNIC_RXD_PKT_OFFSET		0x04
#define AGNIC_RXD_BYTE_CNT		0x06
#define AGNIC_RXD_COOKIE		0x18
#define AGNIC_RXD_SIZE			0x20

#define AGNIC_BPD_BUFF_ADDR_PHYS	0x00
#define AGNIC_BPD_BUFF_COOKIE		0x08
#define AGNIC_BPD_SIZE			0x10

#define AGNIC_COOKIE_DRIVER_WATERMARK	0xdeaddeadULL
#define AGNIC_MAX_QUEUES		(128 + 128 + 128 + 2)
#define AGNIC_RING_INDEX_SLOT_FREE	0xFFFFFFFFU

#define GIU_CMD_Q_LEN		256
#define GIU_NOTIF_Q_LEN		256
#define GIU_DATA_Q_LEN		256
#define GIU_NUM_QS		4
#define GIU_BUF_DEFAULT		2048
#define GIU_BUF_MIN		2048
#define GIU_BUF_MAX		16384
#define GIU_MTU			1500
#define GIU_TAG_LEN		2
#define GIU_META_LEN		64
#define GIU_HDR_LEN		(GIU_TAG_LEN + GIU_META_LEN)
#define GIU_RX_BUDGET		64
#define GIU_TX_WAKE		32

/* ------------------------------------------------------------------------------------------ */
/* NetAgent mailbox (npunwa.h)                                                                 */

#define NWA_COOKIE		0x00
#define  NWA_COOKIE_VALUE	0xCAFEBABEU
#define NWA_BODY_OFF		0x04
#define  NWA_BODY_EXPECTED	0x34
#define NWA_MAX_REQ		0x08
#define NWA_TURN		0x18
#define  NWA_TURN_REQUEST	1
#define  NWA_TURN_ACK		2
#define NWA_REQ_LEN		0x1c
#define NWA_STATUS		0x20
#define  NWA_STATUS_IDLE	0
#define  NWA_STATUS_REPLY	1
#define NWA_REPLY_LEN		0x24
#define NWA_REQ_SIZE		0x20
#define NWA_RP_MARKER		0x00
#define  NWA_RP_MARKER_VALUE	0x14
#define NWA_RP_STATUS		0x04
#define NWA_RP_PAYLOAD		0x08
#define NWA_OP_SET		0x03
#define NWA_OP_GET		0x04
#define NWA_SUB_STATE		0x00
#define NWA_SUB_MAC		0x03
#define NWA_SUB_SPEED		0x04
#define NWA_SUB_MEDIA		0x0a
#define NWA_SUB_PROMISC		0x45
#define NWA_IDLE_WAIT_MS	1000
#define NWA_REPLY_WAIT_MS	30000
#define NWA_READY_TRIES		120	/* x 500 ms */
#define NWA_LINK_TICK_MS	100

/* ------------------------------------------------------------------------------------------ */
/* control-message channel (npuep.h, from usfp_rh.ko debug info)                              */

#define RPC_ST_CFG_MAGIC	0x00
#define  RPC_STATE_CFG_MAGIC	0xD7D3AB00U
#define RPC_ST_CFG_REVISION	0x04
#define  RPC_CFG_REVISION	0x015F
#define RPC_ST_RING_LO		0x48
#define RPC_ST_RINGS		0x68
#define RPC_STATE_SIZE		232
#define RPC_RING_POSTED		0x00
#define RPC_RING_DONE		0x08
#define RPC_RING_OFFSET		0x10
#define RPC_RING_DESC_OFFSET	0x14
#define RPC_RING_DESC_COUNT	0x18
#define RPC_RING_CFG		0x1c
#define RPC_CMD_RESP_BUFF_SZ	0x00
#define RPC_CMD_CMD		0x03
#define RPC_CMD_PAYLOAD		0x08
#define RPC_BD_DMA_ADDR		0x00
#define RPC_BD_PAYLOAD_LEN	0x08
#define  RPC_DESC_NO_AGG_DMA	2
#define RPC_CMD_LIF_ADD_UPDATE	3
#define RPC_CMD_PPORT_UPDATE	5
#define RPC_CMD_LO_LIF_READ	37
#define RPC_RESP_RC		0x00
#define RPC_RESP_DESC_DONE	0x02
#define RPC_RESP_PAYLOAD_LEN	0x06
#define RPC_RESP_PAYLOAD	0x08
#define RPC_LIF_INDEX		0x00
#define  RPC_LIF_IFACE_SHIFT	12
#define RPC_LIF_MAC		0x04
#define RPC_LIF_MTU		0x0a
#define RPC_LIF_FLAGS		0x0c
#define  RPC_LIF_FWD_L2		0x0001
#define  RPC_LIF_OFFLOAD_DISABLED 0x0008
#define RPC_LIF_UPDATE_MASK	0x10
#define  RPC_LIF_CREATE		0x00FF
#define RPC_LIF_REQ_SIZE	18
#define RPC_PPORT_IFACE		0x00
#define RPC_PPORT_TAG		0x02
#define RPC_PPORT_REQ_SIZE	4
#define RPC_TBL_S_INDEX		0x00
#define RPC_TBL_NUM_ENTRIES	0x04
#define RPC_TBL_REQ_SIZE	12
#define RPC_DATA_MAX_SIZE	4096
#define RPC_DESC_OFF		0x1000
#define RPC_DESC_COUNT		32
#define RPC_HI_DESC_OFF		0x2000
#define RPC_HI_DESC_COUNT	32
#define RPC_LIF_MTU_VAL		9000
#define RPC_PROG_RETRY_MS	2000
#define RPC_PROG_TRIES		45

/* ------------------------------------------------------------------------------------------ */
/* the fourteen front ports (npuep.c)                                                          */

struct front_port {
	const char *name;	/* netdev name, from the chassis label */
	u16 tag;		/* the coprocessor's identifier for it */
	u8 iface_id;		/* logical interface the vendor binds it to */
	u8 unit;
	bool fibre;
};

#define NFRONT 14
static const struct front_port front_ports[NFRONT] = {
	{ "port1",  0x8100,  0,  1, false },
	{ "port2",  0x8200,  1,  2, false },
	{ "port3",  0x8300,  2,  3, false },
	{ "port4",  0x8400,  3,  4, false },
	{ "port5",  0x8500,  4,  5, false },
	{ "port6",  0x8600,  5,  6, false },
	{ "port7",  0x8700,  6,  7, false },
	{ "port8",  0x8800,  7,  8, false },
	{ "port9",  0x0001, 10,  9, false },
	{ "port10", 0x0003, 12, 10, false },
	{ "port11", 0x0004, 13, 11, false },
	{ "port12", 0x0002, 11, 12, false },
	{ "portf1", 0x8900,  9, 13, true  },
	{ "portf2", 0x8a00,  8, 14, true  },
};

/* ------------------------------------------------------------------------------------------ */

struct giu_ring {
	void *desc;
	dma_addr_t phys;
	int len;
	int descsz;
	u32 prod_slot;		/* window-relative offset of the producer word */
	u32 cons_slot;
	int prod_idx;
	int cons_idx;
	u32 shadow;		/* the index THIS side owns */
};

struct giu_buf {
	void *vaddr;
	dma_addr_t paddr;
};

struct xgs;

struct xgs_port {
	struct xgs *sc;
	struct net_device *ndev;
	const struct front_port *fp;
	int idx;
	int link;		/* -1 unknown */
	int speed;
	u8 mac[ETH_ALEN];
};

struct nwa_port {
	u8 factory[ETH_ALEN];	/* what NetAgent reported before we set ours */
	bool have_factory;
	int up;
	int link;
	int media;
	int promisc;
};

struct xgs {
	struct pci_dev *pdev;
	struct device *dev;
	void __iomem *bar0, *bar2, *bar4;
	resource_size_t bar0_len, bar2_len, bar4_len;

	/* facility map, absolute offsets in their BAR */
	u32 window;
	u32 ctrl;
	u32 giu_off, giu_size;
	u32 nwa_off, nwa_size;
	u32 rpc_off, rpc_size;
	u32 mgmt_off, mgmt_size;

	int nvec;
	u64 dbell_count[NPU_TOTAL_DBELLS];
	bool handshaken;
	bool lost;
	bool stopping;

	struct workqueue_struct *wq;
	struct work_struct bringup_work;
	struct delayed_work heartbeat_work;
	u32 last_handshake;

	/* ---- giu ---- */
	bool giu_up;
	spinlock_t lock;		/* rings, rx/tx, command post, notif drain */
	struct mutex cmd_mutex;		/* one command in flight */
	wait_queue_head_t cmd_wq;
	struct task_struct *poll_task;
	u32 idx_base;			/* window-relative */
	u32 idx_count;
	struct giu_ring cmd, notif;
	struct giu_ring tx[GIU_NUM_QS], rx[GIU_NUM_QS], bp[GIU_NUM_QS];
	struct giu_buf *buf;		/* GIU_NUM_QS * GIU_DATA_Q_LEN receive buffers */
	int nbuf;
	struct giu_buf *txbuf;		/* one per tx[0] descriptor */
	int ntxbuf;
	int buf_size;
	u32 cap_flags;
	u8 cap_dma_engines;
	bool datapath;
	bool tx_stopped;
	u8 mac[ETH_ALEN];		/* advertised by the device */
	u8 hostmac[ETH_ALEN];
	u16 next_tag;
	bool waiting;
	u16 wait_tag;
	bool answered;
	u8 answer[4 * AGNIC_MGMT_DATA_LEN];
	int answer_len;
	int dev_link;
	u64 commands, answers, notifications, keepalives, late, drops;
	u64 rx_bad, rx_untagged, rx_nomem, tx_full;

	struct xgs_port *port[NFRONT];
	s8 by_hi[16];
	s8 by_lo[8];

	/* ---- netagent ---- */
	struct mutex nwa_mutex;
	struct delayed_work nwa_work;
	bool nwa_ready;
	bool nwa_mailbox;		/* mailbox found and validated */
	bool use_factory;
	int nwa_tries;
	int nwa_sweep;
	u32 nwa_body;
	u32 nwa_max_req;
	struct nwa_port nport[NFRONT];
	u64 nwa_commands, nwa_failures, nwa_timeouts;

	/* ---- rpc ---- */
	struct mutex rpc_mutex;
	struct delayed_work rpc_work;
	void *rpc_cmd;
	dma_addr_t rpc_cmd_pa;
	bool rpc_opened;
	bool rpc_stalled;
	u64 rpc_posted;
	int rpc_tries;
	int rpc_verified;
	bool rpc_announced;
};

/* ------------------------------------------------------------------------------------------ */
/* MMIO helpers                                                                                */

static inline u32 b2_rd(struct xgs *sc, u32 off) { return ioread32(sc->bar2 + off); }
static inline void b2_wr(struct xgs *sc, u32 off, u32 v) { iowrite32(v, sc->bar2 + off); }

static inline u32 cfg_rd(struct xgs *sc, u32 o) { return ioread32(sc->bar0 + sc->giu_off + o); }
static inline void cfg_wr(struct xgs *sc, u32 o, u32 v) { iowrite32(v, sc->bar0 + sc->giu_off + o); }

static inline void idx_publish(struct xgs *sc, u32 slot, u32 v)
{
	wmb();	/* descriptor stores in host memory before the MMIO index */
	iowrite32(v, sc->bar0 + sc->giu_off + slot);
}

/* read a device-owned index, bound it */
static inline bool idx_remote(struct xgs *sc, u32 slot, int len, u32 *out)
{
	u32 v = ioread32(sc->bar0 + sc->giu_off + slot);

	dma_rmb();
	if (v >= (u32)len)
		return false;
	*out = v;
	return true;
}

static inline u8 *desc_at(struct giu_ring *r, u32 i)
{
	return (u8 *)r->desc + (size_t)i * r->descsz;
}

static inline u32 ring_next(struct giu_ring *r, u32 i)
{
	return i == (u32)(r->len - 1) ? 0 : i + 1;
}

static bool xgs_endpoint_alive(struct xgs *sc)
{
	u16 vid = 0xffff;

	pci_read_config_word(sc->pdev, PCI_VENDOR_ID, &vid);
	return vid != 0xffff;
}

/* sleep in small steps, give up early when the driver is going away */
static bool xgs_sleep(struct xgs *sc, unsigned int ms)
{
	if (READ_ONCE(sc->stopping))
		return false;
	msleep(ms);
	return !READ_ONCE(sc->stopping);
}

/* ------------------------------------------------------------------------------------------ */
/* barmap, doorbells, handshake                                                                */

static int xgs_read_barmap(struct xgs *sc, bool verbose)
{
	u32 base = sc->window + NPU_BARMAP_STRUCT_OFF;
	u32 version = b2_rd(sc, base), cookie = b2_rd(sc, base + 4);
	bool have_ctrl = false;
	int i;

	if (cookie == 0xffffffffU) {
		dev_err(sc->dev, "BAR2 reads all ones - endpoint not decoding (NPU reset? BARs not restored?)\n");
		return -ENXIO;
	}
	if (cookie != NPU_BARMAP_COOKIE)
		return -EAGAIN;
	if (version != NPU_BARMAP_VERSION) {
		dev_err(sc->dev, "barmap version %u, expected %u\n", version, NPU_BARMAP_VERSION);
		return -EINVAL;
	}

	for (i = 0; i < FAC_COUNT; i++) {
		u32 e = base + 8 + i * 16;
		u32 bar = b2_rd(sc, e), type = b2_rd(sc, e + 4);
		u32 off = b2_rd(sc, e + 8), size = b2_rd(sc, e + 12);

		if (!bar && !type && !off && !size)
			continue;	/* not published yet */
		if (verbose)
			dev_info(sc->dev, "  facility %-6s bar%u off 0x%06x size 0x%x\n",
				 type < FAC_COUNT ? fac_name[type] : "?", bar ? 2 : 0, off, size);

		/* bound every device-supplied offset against what is mapped: size first */
		if (bar == 1) {
			if (size > NPU_BARMAP_WINDOW_LEN || off > NPU_BARMAP_WINDOW_LEN - size || (off & 7))
				continue;
		} else if (bar == 0) {
			if (size > sc->bar0_len || off > sc->bar0_len - size || (off & 7))
				continue;
		} else {
			continue;
		}

		switch (type) {
		case FAC_CTRL:
			if (bar != 1 || off > NPU_BARMAP_WINDOW_LEN - 0x100)
				return -EINVAL;
			sc->ctrl = sc->window + off;
			have_ctrl = true;
			break;
		case FAC_MGMT:
			if (bar == 1) {
				sc->mgmt_off = sc->window + off;
				sc->mgmt_size = size;
			}
			break;
		case FAC_GIU:
			if (bar == 0 && size >= AGNIC_CFG_SIZE) {
				sc->giu_off = off;
				sc->giu_size = size;
			}
			break;
		case FAC_NWA:
			if (bar == 0 && size >= 0x100) {
				sc->nwa_off = off;
				sc->nwa_size = size;
			}
			break;
		case FAC_RPC:
			if (bar == 1 && size >= RPC_DESC_OFF + RPC_DESC_COUNT * 16) {
				sc->rpc_off = sc->window + off;
				sc->rpc_size = size;
			}
			break;
		}
	}
	return have_ctrl ? 0 : -EAGAIN;
}

static int xgs_wait_barmap(struct xgs *sc)
{
	int i, err = -EAGAIN;

	for (i = 0; i <= 120; i++) {
		err = xgs_read_barmap(sc, false);
		if (err && err != -EAGAIN)
			return err;
		if (!err && sc->giu_size && sc->nwa_size && sc->rpc_size)
			break;
		if (i == 0)
			dev_info(sc->dev, "waiting up to 120 s for the NPU to publish its facility table\n");
		if (!xgs_sleep(sc, 1000))
			return -EINTR;
	}
	err = xgs_read_barmap(sc, true);
	if (err == -EAGAIN) {
		dev_err(sc->dev, "the NPU never published a control facility - is it running?\n");
		return -ENXIO;
	}
	return err;
}

static int xgs_ring_dbell(struct xgs *sc, int n)
{
	u32 at, cnt, data;
	u64 off;

	if (!sc->bar4)
		return -ENXIO;
	cnt = b2_rd(sc, sc->ctrl + CTRL_H2T_DBELL_CNT);
	if (n < 0 || cnt > CTRL_DBELL_MAX || (u32)n >= cnt)
		return -EINVAL;
	at = sc->ctrl + CTRL_H2T_DBELL_MSG + n * CTRL_DBELL_MSG_SIZE;
	off = (u64)b2_rd(sc, at) | ((u64)b2_rd(sc, at + 4) << 32);
	data = b2_rd(sc, at + 8);
	if (!off)
		return -ENXIO;
	if ((off & 3) || off > (u64)sc->bar4_len - 4) {
		dev_warn(sc->dev, "doorbell %d offset %#llx does not fit BAR4\n", n, off);
		return -ERANGE;
	}
	wmb();
	iowrite32(data, sc->bar4 + off);
	return 0;
}

static irqreturn_t xgs_dbell_isr(int irq, void *arg)
{
	u64 *count = arg;

	(*count)++;
	return IRQ_HANDLED;
}

static int xgs_setup_msix(struct xgs *sc)
{
	int i, n, err;

	n = pci_alloc_irq_vectors(sc->pdev, NPU_TOTAL_DBELLS, NPU_TOTAL_DBELLS, PCI_IRQ_MSIX);
	if (n < 0) {
		dev_err(sc->dev, "cannot allocate %d MSI-X vectors: %d\n", NPU_TOTAL_DBELLS, n);
		return n;
	}
	for (i = 0; i < NPU_TOTAL_DBELLS; i++) {
		err = request_irq(pci_irq_vector(sc->pdev, i), xgs_dbell_isr, 0, DRV_NAME,
				  &sc->dbell_count[i]);
		if (err) {
			while (--i >= 0)
				free_irq(pci_irq_vector(sc->pdev, i), &sc->dbell_count[i]);
			pci_free_irq_vectors(sc->pdev);
			return err;
		}
	}
	sc->nvec = NPU_TOTAL_DBELLS;
	dev_info(sc->dev, "%d MSI-X doorbells armed (mvmgmt x1, giu x4)\n", sc->nvec);
	return 0;
}

static void xgs_free_msix(struct xgs *sc)
{
	int i;

	for (i = 0; i < sc->nvec; i++)
		free_irq(pci_irq_vector(sc->pdev, i), &sc->dbell_count[i]);
	if (sc->nvec)
		pci_free_irq_vectors(sc->pdev);
	sc->nvec = 0;
}

static int xgs_handshake(struct xgs *sc)
{
	u32 hs = 0;
	int i;

	if (b2_rd(sc, sc->ctrl) != ARMADA_FACILITY_COOKIE) {
		dev_err(sc->dev, "control facility cookie %#x - refusing to write\n", b2_rd(sc, sc->ctrl));
		return -EINVAL;
	}
	for (i = 0; i < 100; i++) {
		hs = b2_rd(sc, sc->ctrl + CTRL_HANDSHAKE);
		if (hs & CTRL_TRGT_INIT)
			break;
		if (!xgs_sleep(sc, 100))
			return -EINTR;
	}
	if (!(hs & CTRL_TRGT_INIT)) {
		dev_err(sc->dev, "target never set TRGT_INIT (handshake %#x)\n", hs);
		return -ETIMEDOUT;
	}
	wmb();
	b2_wr(sc, sc->ctrl + CTRL_HANDSHAKE, hs | CTRL_HOST_INIT);
	sc->handshaken = true;
	hs = b2_rd(sc, sc->ctrl + CTRL_HANDSHAKE);
	b2_wr(sc, sc->ctrl + CTRL_HANDSHAKE, hs | CTRL_HOST_ALIVE);
	hs = b2_rd(sc, sc->ctrl + CTRL_HANDSHAKE);
	dev_info(sc->dev, "handshake %#x%s\n", hs,
		 (hs & 0x0b) == 0x0b ? " - target may proceed" : "");
	return 0;
}

static void xgs_withdraw_handshake(struct xgs *sc)
{
	u32 hs;

	if (!sc->handshaken || sc->lost)
		return;
	hs = b2_rd(sc, sc->ctrl + CTRL_HANDSHAKE);
	if (hs != 0xffffffffU)
		b2_wr(sc, sc->ctrl + CTRL_HANDSHAKE, hs & ~(CTRL_HOST_INIT | CTRL_HOST_ALIVE));
	sc->handshaken = false;
}

/* the target clears HOST_ALIVE on every scan; keep setting it */
static void xgs_heartbeat(struct work_struct *w)
{
	struct xgs *sc = container_of(to_delayed_work(w), struct xgs, heartbeat_work);
	u32 hs;

	if (READ_ONCE(sc->stopping) || !sc->handshaken)
		return;
	hs = b2_rd(sc, sc->ctrl + CTRL_HANDSHAKE);
	if (hs == 0xffffffffU) {
		if (!sc->lost)
			dev_err(sc->dev, "endpoint stopped responding - NPU reset? heartbeat stopped\n");
		sc->lost = true;
		return;
	}
	b2_wr(sc, sc->ctrl + CTRL_HANDSHAKE, hs | CTRL_HOST_ALIVE);
	sc->last_handshake = hs;
	queue_delayed_work(sc->wq, &sc->heartbeat_work, HZ);
}

/* ------------------------------------------------------------------------------------------ */
/* GIU: index slots and rings                                                                  */

static void giu_init_slots(struct xgs *sc)
{
	u32 i;

	for (i = 0; i < sc->idx_count; i++)
		cfg_wr(sc, sc->idx_base + i * 4, AGNIC_RING_INDEX_SLOT_FREE);
}

static int giu_claim_slot(struct xgs *sc, u32 *slot)
{
	u32 i, rel;

	for (i = 0; i < sc->idx_count; i++) {
		rel = sc->idx_base + i * 4;
		if (cfg_rd(sc, rel) == AGNIC_RING_INDEX_SLOT_FREE) {
			cfg_wr(sc, rel, 0);
			*slot = rel;
			return i;
		}
	}
	return -1;
}

static void giu_release_slot(struct xgs *sc, int which)
{
	if (which >= 0)
		cfg_wr(sc, sc->idx_base + which * 4, AGNIC_RING_INDEX_SLOT_FREE);
}

static int giu_alloc_ring(struct xgs *sc, struct giu_ring *r, int len, int descsz, const char *what)
{
	r->len = len;
	r->descsz = descsz;
	r->shadow = 0;
	r->prod_idx = r->cons_idx = -1;
	r->desc = dma_alloc_coherent(sc->dev, (size_t)len * descsz, &r->phys, GFP_KERNEL);
	if (!r->desc)
		return -ENOMEM;
	r->prod_idx = giu_claim_slot(sc, &r->prod_slot);
	r->cons_idx = giu_claim_slot(sc, &r->cons_slot);
	if (r->prod_idx < 0 || r->cons_idx < 0) {
		dev_err(sc->dev, "giu: no free index slot for the %s ring\n", what);
		return -ENOSPC;
	}
	return 0;
}

static void giu_free_ring(struct xgs *sc, struct giu_ring *r)
{
	giu_release_slot(sc, r->prod_idx);
	giu_release_slot(sc, r->cons_idx);
	r->prod_idx = r->cons_idx = -1;
	if (r->desc)
		dma_free_coherent(sc->dev, (size_t)r->len * r->descsz, r->desc, r->phys);
	r->desc = NULL;
}

static void giu_publish_ring(struct xgs *sc, u32 at, struct giu_ring *r)
{
	cfg_wr(sc, at + AGNIC_QI_ADDR, lower_32_bits(r->phys));
	cfg_wr(sc, at + AGNIC_QI_ADDR + 4, upper_32_bits(r->phys));
	cfg_wr(sc, at + AGNIC_QI_PROD_OFFS, r->prod_slot);
	cfg_wr(sc, at + AGNIC_QI_CONS_OFFS, r->cons_slot);
	cfg_wr(sc, at + AGNIC_QI_LEN, r->len);
	cfg_wr(sc, at + AGNIC_QI_RES, 0);
}

/* ------------------------------------------------------------------------------------------ */
/* GIU: command channel                                                                        */

static u16 giu_next_tag(struct xgs *sc)
{
	do {
		sc->next_tag++;
	} while (sc->next_tag == AGNIC_CMD_ID_ILLEGAL || sc->next_tag == AGNIC_CMD_ID_NOTIFICATION);
	return sc->next_tag;
}

/* called with sc->lock held */
static int giu_post(struct xgs *sc, u8 code, const void *params, size_t plen)
{
	struct giu_ring *r = &sc->cmd;
	u32 cons, push;
	u8 *d;

	if (plen > AGNIC_MGMT_DATA_LEN)
		return -EMSGSIZE;
	if (!idx_remote(sc, r->cons_slot, r->len, &cons))
		return -EIO;
	push = r->shadow;
	if (ring_next(r, push) == cons)
		return -EBUSY;

	d = desc_at(r, push);
	memset(d, 0, AGNIC_CMD_DESC_SIZE);
	sc->wait_tag = giu_next_tag(sc);
	put_unaligned_le16(sc->wait_tag, d + AGNIC_CMD_IDX);
	put_unaligned_le16(AGNIC_AC_PF_MANAGER, d + AGNIC_CMD_APP_CODE);
	d[AGNIC_CMD_CODE] = code;
	d[AGNIC_CMD_CLIENT_ID] = 0;
	d[AGNIC_CMD_CLIENT_TYPE] = AGNIC_CDT_PF;
	d[AGNIC_CMD_FLAGS] = AGNIC_BUF_POS_SINGLE << AGNIC_F_BUF_POS_SHIFT;
	if (params && plen)
		memcpy(d + AGNIC_CMD_DATA, params, plen);

	r->shadow = ring_next(r, push);
	idx_publish(sc, r->prod_slot, r->shadow);

	sc->commands++;
	sc->waiting = true;
	sc->answered = false;
	sc->answer_len = 0;
	return 0;
}

/* drain the notification ring: answers and async notifications. sc->lock held. */
static void giu_drain(struct xgs *sc)
{
	struct giu_ring *r = &sc->notif;
	int guard = r->len;
	u32 prod;

	if (!idx_remote(sc, r->prod_slot, r->len, &prod))
		return;

	while (r->shadow != prod && guard-- > 0) {
		u8 *d = desc_at(r, r->shadow);
		u16 tag = get_unaligned_le16(d + AGNIC_CMD_IDX);
		u8 code = d[AGNIC_CMD_CODE];

		if (tag == AGNIC_CMD_ID_NOTIFICATION) {
			sc->notifications++;
			if (code == AGNIC_NC_PF_KEEP_ALIVE) {
				sc->keepalives++;
			} else if (code == AGNIC_NC_PF_LINK_CHANGE) {
				sc->dev_link = get_unaligned_le32(d + AGNIC_CMD_DATA);
				dev_info(sc->dev, "giu: device reports the host link %s\n",
					 sc->dev_link ? "up" : "down");
			}
		} else if (!sc->waiting && tag == sc->wait_tag) {
			sc->late++;
		} else if (sc->waiting && tag == sc->wait_tag) {
			int pos = (d[AGNIC_CMD_FLAGS] >> AGNIC_F_BUF_POS_SHIFT) & AGNIC_F_BUF_POS_MASK;

			if (sc->answer_len + AGNIC_MGMT_DATA_LEN <= (int)sizeof(sc->answer)) {
				memcpy(sc->answer + sc->answer_len, d + AGNIC_CMD_DATA, AGNIC_MGMT_DATA_LEN);
				sc->answer_len += AGNIC_MGMT_DATA_LEN;
			}
			if (pos == AGNIC_BUF_POS_SINGLE || pos == AGNIC_BUF_POS_LAST) {
				sc->waiting = false;
				sc->answers++;
				WRITE_ONCE(sc->answered, true);
				wake_up(&sc->cmd_wq);
			}
		} else {
			sc->drops++;
		}
		r->shadow = ring_next(r, r->shadow);
		idx_publish(sc, r->cons_slot, r->shadow);
	}
}

/*
 * Post a command and wait for its answer; check the status byte. On success the answer
 * stays in sc->answer until the next command (cmd_mutex held by caller).
 */
static int giu_command_locked(struct xgs *sc, u8 code, const void *params, size_t plen)
{
	long left;
	int err;

	spin_lock_bh(&sc->lock);
	err = giu_post(sc, code, params, plen);
	spin_unlock_bh(&sc->lock);
	if (err)
		return err;

	left = wait_event_timeout(sc->cmd_wq, READ_ONCE(sc->answered) || READ_ONCE(sc->stopping),
				  5 * HZ);
	spin_lock_bh(&sc->lock);
	if (!sc->answered) {
		sc->waiting = false;
		spin_unlock_bh(&sc->lock);
		return left ? -EINTR : -ETIMEDOUT;
	}
	spin_unlock_bh(&sc->lock);

	if (sc->answer_len < 1)
		return -EBADMSG;
	if (sc->answer[AGNIC_R_STATUS] != 0) {
		dev_warn(sc->dev, "giu: command 0x%02x refused, status %u\n", code,
			 sc->answer[AGNIC_R_STATUS]);
		return -EINVAL;
	}
	return 0;
}

static int giu_command(struct xgs *sc, u8 code, const void *params, size_t plen)
{
	int err;

	mutex_lock(&sc->cmd_mutex);
	err = giu_command_locked(sc, code, params, plen);
	mutex_unlock(&sc->cmd_mutex);
	return err;
}

/* ------------------------------------------------------------------------------------------ */
/* GIU: datapath                                                                               */

static inline int giu_port_of_tag(struct xgs *sc, u16 tag)
{
	if (tag & 0x8000) {
		int n = (tag >> 8) & 0x7f;

		return n < 16 ? sc->by_hi[n] : -1;
	}
	return tag < 8 ? sc->by_lo[tag] : -1;
}

static void giu_bpool_return(struct xgs *sc, u32 bufidx)
{
	struct giu_ring *r;
	u32 cons, push, q;
	u8 *d;

	if (bufidx >= (u32)sc->nbuf)
		return;
	q = bufidx / GIU_DATA_Q_LEN;
	if (q >= GIU_NUM_QS)
		return;
	r = &sc->bp[q];
	if (!idx_remote(sc, r->cons_slot, r->len, &cons))
		return;
	push = r->shadow;
	if (ring_next(r, push) == cons)
		return;
	d = desc_at(r, push);
	put_unaligned_le64(sc->buf[bufidx].paddr, d + AGNIC_BPD_BUFF_ADDR_PHYS);
	put_unaligned_le64(bufidx, d + AGNIC_BPD_BUFF_COOKIE);
	r->shadow = ring_next(r, push);
	idx_publish(sc, r->prod_slot, r->shadow);
}

/* sc->lock held; delivered skbs are queued on @list for the caller */
static int giu_rx_queue(struct xgs *sc, int q, int budget, struct sk_buff_head *list)
{
	struct giu_ring *r = &sc->rx[q];
	u32 prod;
	int done = 0;

	if (!idx_remote(sc, r->prod_slot, r->len, &prod))
		return 0;

	while (done < budget && r->shadow != prod) {
		u8 *d = desc_at(r, r->shadow);
		u64 cookie = get_unaligned_le64(d + AGNIC_RXD_COOKIE);
		u16 total = get_unaligned_le16(d + AGNIC_RXD_BYTE_CNT);
		u32 bufidx = (u32)cookie;
		const u8 *src;
		struct sk_buff *skb;
		int off, n, len;
		u16 tag;

		if (cookie == AGNIC_COOKIE_DRIVER_WATERMARK || bufidx >= (u32)sc->nbuf) {
			sc->rx_bad++;
			goto next;
		}
		off = d[AGNIC_RXD_PKT_OFFSET];
		if (off + (int)total > sc->buf_size || total < GIU_HDR_LEN + ETH_HLEN) {
			sc->rx_bad++;
			giu_bpool_return(sc, bufidx);
			goto next;
		}
		src = (const u8 *)sc->buf[bufidx].vaddr + off;
		tag = ((u16)src[0] << 8) | src[1];
		len = total - GIU_HDR_LEN;
		n = giu_port_of_tag(sc, tag);
		if (n < 0 || !sc->port[n]) {
			sc->rx_untagged++;
			goto consumed;
		}
		skb = netdev_alloc_skb_ip_align(sc->port[n]->ndev, len);
		if (!skb) {
			/* leave the descriptor intact; retry next pass */
			sc->rx_nomem++;
			break;
		}
		skb_put_data(skb, src + GIU_HDR_LEN, len);
		skb->protocol = eth_type_trans(skb, sc->port[n]->ndev);
		sc->port[n]->ndev->stats.rx_packets++;
		sc->port[n]->ndev->stats.rx_bytes += len;
		__skb_queue_tail(list, skb);
consumed:
		put_unaligned_le64(AGNIC_COOKIE_DRIVER_WATERMARK, d + AGNIC_RXD_COOKIE);
		giu_bpool_return(sc, bufidx);
next:
		r->shadow = ring_next(r, r->shadow);
		idx_publish(sc, r->cons_slot, r->shadow);
		done++;
	}
	return done;
}

static int giu_tx_free(struct xgs *sc)
{
	struct giu_ring *r = &sc->tx[0];
	u32 cons;

	if (!idx_remote(sc, r->cons_slot, r->len, &cons))
		return 0;
	return (int)((cons + r->len - r->shadow - 1) % r->len);
}

static int giu_poll_thread(void *arg)
{
	struct xgs *sc = arg;
	struct sk_buff_head list;
	int idle = 0;

	__skb_queue_head_init(&list);
	while (!kthread_should_stop()) {
		struct sk_buff *skb;
		int done = 0, q, i;
		bool wake = false;

		spin_lock_bh(&sc->lock);
		giu_drain(sc);
		if (sc->datapath) {
			for (q = 0; q < GIU_NUM_QS; q++)
				done += giu_rx_queue(sc, q, GIU_RX_BUDGET, &list);
			if (sc->tx_stopped && giu_tx_free(sc) >= GIU_TX_WAKE) {
				sc->tx_stopped = false;
				wake = true;
			}
		}
		spin_unlock_bh(&sc->lock);

		if (wake)
			for (i = 0; i < NFRONT; i++)
				if (sc->port[i])
					netif_wake_queue(sc->port[i]->ndev);

		if (!skb_queue_empty(&list)) {
			local_bh_disable();
			while ((skb = __skb_dequeue(&list)))
				netif_receive_skb(skb);
			local_bh_enable();
		}

		if (done) {
			idle = 0;
			cond_resched();
		} else if (++idle < 1000) {
			usleep_range(poll_us, poll_us * 2);
		} else {
			usleep_range(1000, 2000);	/* quiet link: back off */
		}
	}
	return 0;
}

static netdev_tx_t xgs_start_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct xgs_port *pt = netdev_priv(ndev);
	struct xgs *sc = pt->sc;
	struct giu_ring *r = &sc->tx[0];
	unsigned int len = skb->len, wlen;
	u32 cons, push;
	u8 *b, *d;
	int i;

	if (len + GIU_HDR_LEN > (unsigned int)sc->buf_size) {
		ndev->stats.tx_dropped++;
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}

	spin_lock_bh(&sc->lock);
	if (!sc->datapath || !idx_remote(sc, r->cons_slot, r->len, &cons)) {
		spin_unlock_bh(&sc->lock);
		ndev->stats.tx_dropped++;
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}
	push = r->shadow;
	if (ring_next(r, push) == cons) {
		sc->tx_full++;
		sc->tx_stopped = true;
		spin_unlock_bh(&sc->lock);
		for (i = 0; i < NFRONT; i++)
			if (sc->port[i])
				netif_stop_queue(sc->port[i]->ndev);
		return NETDEV_TX_BUSY;
	}

	b = sc->txbuf[push].vaddr;
	b[0] = pt->fp->tag >> 8;
	b[1] = pt->fp->tag & 0xff;
	memset(b + GIU_TAG_LEN, 0, GIU_META_LEN);
	skb_copy_bits(skb, 0, b + GIU_HDR_LEN, len);
	wlen = len;
	if (wlen < ETH_ZLEN) {
		memset(b + GIU_HDR_LEN + wlen, 0, ETH_ZLEN - wlen);
		wlen = ETH_ZLEN;
	}

	d = desc_at(r, push);
	memset(d, 0, AGNIC_TXD_SIZE);
	put_unaligned_le32(AGNIC_TXD_F_MD_MODE | AGNIC_TXD_F_GEN_L4_CSUM_NOT |
			   AGNIC_TXD_F_GEN_IPV4_CSUM_DIS, d + AGNIC_TXD_FLAGS);
	d[AGNIC_TXD_PKT_OFFSET] = 0;
	put_unaligned_le16(wlen + GIU_HDR_LEN, d + AGNIC_TXD_BYTE_CNT);
	put_unaligned_le64(sc->txbuf[push].paddr, d + AGNIC_TXD_BUFFER_ADDR);
	put_unaligned_le64(push, d + AGNIC_TXD_COOKIE);

	r->shadow = ring_next(r, push);
	idx_publish(sc, r->prod_slot, r->shadow);
	ndev->stats.tx_packets++;
	ndev->stats.tx_bytes += len;
	spin_unlock_bh(&sc->lock);

	dev_consume_skb_any(skb);
	return NETDEV_TX_OK;
}

static int xgs_open(struct net_device *ndev)
{
	struct xgs_port *pt = netdev_priv(ndev);

	/* the network agent reports changes only, so restore what it last said */
	if (pt->link > 0)
		netif_carrier_on(ndev);
	else
		netif_carrier_off(ndev);
	netif_start_queue(ndev);
	return 0;
}

static int xgs_stop(struct net_device *ndev)
{
	netif_stop_queue(ndev);
	return 0;
}

static int xgs_change_mtu(struct net_device *ndev, int mtu)
{
	WRITE_ONCE(ndev->mtu, mtu);
	return 0;
}

static const struct net_device_ops xgs_netdev_ops = {
	.ndo_open		= xgs_open,
	.ndo_stop		= xgs_stop,
	.ndo_start_xmit		= xgs_start_xmit,
	.ndo_change_mtu		= xgs_change_mtu,
	.ndo_validate_addr	= eth_validate_addr,
};

static void xgs_get_drvinfo(struct net_device *ndev, struct ethtool_drvinfo *info)
{
	struct xgs_port *pt = netdev_priv(ndev);

	strscpy(info->driver, DRV_NAME, sizeof(info->driver));
	strscpy(info->bus_info, pci_name(pt->sc->pdev), sizeof(info->bus_info));
	snprintf(info->fw_version, sizeof(info->fw_version), "tag 0x%04x", pt->fp->tag);
}

static int xgs_get_link_ksettings(struct net_device *ndev, struct ethtool_link_ksettings *ks)
{
	struct xgs_port *pt = netdev_priv(ndev);

	ethtool_link_ksettings_zero_link_mode(ks, supported);
	ethtool_link_ksettings_zero_link_mode(ks, advertising);
	ks->base.port = pt->fp->fibre ? PORT_FIBRE : PORT_TP;
	ks->base.autoneg = AUTONEG_ENABLE;
	if (pt->link > 0 && pt->speed > 0) {
		ks->base.speed = pt->speed;
		ks->base.duplex = DUPLEX_FULL;
	} else {
		ks->base.speed = SPEED_UNKNOWN;
		ks->base.duplex = DUPLEX_UNKNOWN;
	}
	return 0;
}

static const struct ethtool_ops xgs_ethtool_ops = {
	.get_drvinfo		= xgs_get_drvinfo,
	.get_link		= ethtool_op_get_link,
	.get_link_ksettings	= xgs_get_link_ksettings,
};

static void giu_free_buffers(struct xgs *sc)
{
	int i;

	if (sc->buf) {
		for (i = 0; i < sc->nbuf; i++)
			if (sc->buf[i].vaddr)
				dma_free_coherent(sc->dev, sc->buf_size, sc->buf[i].vaddr, sc->buf[i].paddr);
		kfree(sc->buf);
		sc->buf = NULL;
	}
	if (sc->txbuf) {
		for (i = 0; i < sc->ntxbuf; i++)
			if (sc->txbuf[i].vaddr)
				dma_free_coherent(sc->dev, sc->buf_size, sc->txbuf[i].vaddr, sc->txbuf[i].paddr);
		kfree(sc->txbuf);
		sc->txbuf = NULL;
	}
	sc->nbuf = sc->ntxbuf = 0;
}

static int giu_alloc_bufs(struct xgs *sc, struct giu_buf **arr, int *cnt, int n)
{
	int i;

	*arr = kcalloc(n, sizeof(**arr), GFP_KERNEL);
	if (!*arr)
		return -ENOMEM;
	*cnt = n;
	for (i = 0; i < n; i++) {
		(*arr)[i].vaddr = dma_alloc_coherent(sc->dev, sc->buf_size, &(*arr)[i].paddr, GFP_KERNEL);
		if (!(*arr)[i].vaddr)
			return -ENOMEM;
	}
	return 0;
}

static void giu_capabilities(struct xgs *sc)
{
	u8 p[AGNIC_MGMT_DATA_LEN] = { 0 };
	u32 want;

	sc->buf_size = GIU_BUF_DEFAULT;
	mutex_lock(&sc->cmd_mutex);
	if (giu_command_locked(sc, AGNIC_CC_GET_CAPABILITIES, p, 0) ||
	    sc->answer_len < AGNIC_R_CAP_EGRESS_DMA + 1) {
		mutex_unlock(&sc->cmd_mutex);
		dev_warn(sc->dev, "giu: no capabilities - using %d byte buffers\n", sc->buf_size);
		return;
	}
	sc->cap_flags = get_unaligned_le32(sc->answer + AGNIC_R_CAP_FLAGS);
	sc->cap_dma_engines = sc->answer[AGNIC_R_CAP_EGRESS_DMA];
	want = get_unaligned_le32(sc->answer + AGNIC_R_CAP_MAX_BUF_SIZE);
	mutex_unlock(&sc->cmd_mutex);
	if (want >= GIU_BUF_MIN && want <= GIU_BUF_MAX)
		sc->buf_size = want;
	dev_info(sc->dev, "giu: capabilities flags %#x, buffer %u bytes (using %d), %u egress DMA engine(s)\n",
		 sc->cap_flags, want, sc->buf_size, sc->cap_dma_engines);
}

static void giu_fill_bpool(struct xgs *sc)
{
	int q, i, total = 0;

	spin_lock_bh(&sc->lock);
	for (q = 0; q < GIU_NUM_QS; q++) {
		struct giu_ring *r = &sc->bp[q];
		int base = q * GIU_DATA_Q_LEN, fill = r->len - 1;

		if (base + fill > sc->nbuf)
			fill = max(0, sc->nbuf - base);
		for (i = 0; i < fill; i++) {
			u8 *d = desc_at(r, i);

			put_unaligned_le64(sc->buf[base + i].paddr, d + AGNIC_BPD_BUFF_ADDR_PHYS);
			put_unaligned_le64(base + i, d + AGNIC_BPD_BUFF_COOKIE);
		}
		cfg_wr(sc, r->cons_slot, 0);
		r->shadow = fill;
		idx_publish(sc, r->prod_slot, r->shadow);
		total += fill;
	}
	spin_unlock_bh(&sc->lock);
	dev_info(sc->dev, "giu: %d buffers of %d B across %d pools\n", total, sc->buf_size, GIU_NUM_QS);
}

static int giu_bringup(struct xgs *sc)
{
	u8 p[AGNIC_MGMT_DATA_LEN];
	int err, q;

#define STEP(code, len, what) do {					\
		err = giu_command(sc, (code), p, (len));		\
		if (err) {						\
			dev_err(sc->dev, "giu: %s failed (%d)\n", (what), err); \
			return err;					\
		}							\
	} while (0)
#define SOFT_STEP(code, len, what) do {					\
		int serr = giu_command(sc, (code), p, (len));		\
		if (serr)						\
			dev_warn(sc->dev, "giu: %s failed (%d) - carrying on\n", (what), serr); \
	} while (0)

	memset(p, 0, sizeof(p));
	put_unaligned_le32(1, p + 0x00);		/* num egress tc */
	put_unaligned_le32(1, p + 0x04);		/* num ingress tc */
	put_unaligned_le16(GIU_MTU, p + 0x08);		/* mtu override */
	put_unaligned_le16(GIU_MTU, p + 0x0a);		/* mru override */
	p[0x0c] = AGNIC_ES_STRICT_SCHED;
	STEP(AGNIC_CC_PF_INIT, 0x10, "PF_INIT");

	memset(p, 0, sizeof(p));
	put_unaligned_le32(0, p + 0x00);		/* tc */
	put_unaligned_le32(GIU_NUM_QS, p + 0x04);	/* num queues */
	put_unaligned_le32(0, p + 0x08);		/* pkt offset */
	p[0x0c] = GIU_NUM_QS > 1 ? AGNIC_ING_HASH_2_TUPLE : AGNIC_ING_HASH_NONE;
	STEP(AGNIC_CC_PF_INGRESS_TC_ADD, 0x10, "INGRESS_TC_ADD");

	for (q = 0; q < GIU_NUM_QS; q++) {
		memset(p, 0, sizeof(p));
		put_unaligned_le64(sc->rx[q].phys, p + 0x00);
		put_unaligned_le32(sc->rx[q].prod_slot, p + 0x08);
		put_unaligned_le32(sc->rx[q].cons_slot, p + 0x0c);
		put_unaligned_le64(sc->bp[q].phys, p + 0x10);
		put_unaligned_le32(sc->bp[q].prod_slot, p + 0x18);
		put_unaligned_le32(sc->bp[q].cons_slot, p + 0x1c);
		put_unaligned_le32(sc->rx[q].len, p + 0x20);
		put_unaligned_le32(0, p + 0x24);	/* msix id: polled */
		put_unaligned_le32(0, p + 0x28);	/* tc */
		put_unaligned_le32(sc->buf_size, p + 0x2c);
		STEP(AGNIC_CC_PF_INGRESS_DATA_Q_ADD, 0x30, "INGRESS_DATA_Q_ADD");
	}

	memset(p, 0, sizeof(p));
	put_unaligned_le32(0, p + 0x00);
	put_unaligned_le32(GIU_NUM_QS * (sc->cap_dma_engines ? sc->cap_dma_engines : 1), p + 0x04);
	put_unaligned_le32(GIU_NUM_QS, p + 0x08);
	STEP(AGNIC_CC_PF_EGRESS_TC_ADD, 0x0c, "EGRESS_TC_ADD");

	for (q = 0; q < GIU_NUM_QS; q++) {
		memset(p, 0, sizeof(p));
		put_unaligned_le64(sc->tx[q].phys, p + 0x00);
		put_unaligned_le32(sc->tx[q].prod_slot, p + 0x08);
		put_unaligned_le32(sc->tx[q].cons_slot, p + 0x0c);
		put_unaligned_le32(sc->tx[q].len, p + 0x10);
		put_unaligned_le32(0, p + 0x14);	/* wrr weight: strict prio */
		put_unaligned_le32(0, p + 0x18);	/* tc */
		put_unaligned_le32(0, p + 0x1c);	/* msix id */
		STEP(AGNIC_CC_PF_EGRESS_DATA_Q_ADD, 0x20, "EGRESS_DATA_Q_ADD");
	}

	memset(p, 0, sizeof(p));
	STEP(AGNIC_CC_PF_INIT_DONE, 0, "INIT_DONE");

	giu_fill_bpool(sc);

	memset(p, 0, sizeof(p));
	put_unaligned_le16(GIU_MTU, p);
	SOFT_STEP(AGNIC_CC_PF_MTU, 2, "PF_MTU");

	memset(p, 0, sizeof(p));
	memcpy(p, sc->hostmac, ETH_ALEN);
	SOFT_STEP(AGNIC_CC_PF_MAC_ADDR, ETH_ALEN, "PF_MAC_ADDR");

	memset(p, 0, sizeof(p));
	p[0] = 1;
	SOFT_STEP(AGNIC_CC_PF_PROMISC, 1, "PF_PROMISC");
	memset(p, 0, sizeof(p));
	p[0] = 1;
	SOFT_STEP(AGNIC_CC_PF_MC_PROMISC, 1, "PF_MC_PROMISC");

	memset(p, 0, sizeof(p));
	STEP(AGNIC_CC_PF_ENABLE, 0, "PF_ENABLE");

	mutex_lock(&sc->cmd_mutex);
	memset(p, 0, sizeof(p));
	if (!giu_command_locked(sc, AGNIC_CC_PF_LINK_STATUS, p, 0) &&
	    sc->answer_len >= AGNIC_R_LINK_STATUS + 4) {
		sc->dev_link = get_unaligned_le32(sc->answer + AGNIC_R_LINK_STATUS);
		dev_info(sc->dev, "giu: device reports the host link %s\n", sc->dev_link ? "up" : "down");
	}
	mutex_unlock(&sc->cmd_mutex);
#undef SOFT_STEP
#undef STEP
	spin_lock_bh(&sc->lock);
	sc->datapath = true;
	spin_unlock_bh(&sc->lock);
	return 0;
}

static int giu_wait_status(struct xgs *sc, u32 bit, int ms, const char *what)
{
	u32 st = 0;
	int i;

	for (i = 0; i < ms / 10; i++) {
		st = cfg_rd(sc, AGNIC_CFG_STATUS);
		if (st == 0xffffffffU) {
			dev_err(sc->dev, "giu: endpoint not decoding\n");
			return -ENXIO;
		}
		if (st & bit)
			return 0;
		if (!xgs_sleep(sc, 10))
			return -EINTR;
	}
	dev_err(sc->dev, "giu: timed out waiting for %s (status %#x)\n", what, st);
	return -ETIMEDOUT;
}

static int giu_register_netdevs(struct xgs *sc)
{
	int i, err;

	for (i = 0; i < NFRONT; i++) {
		const struct front_port *fp = &front_ports[i];
		struct net_device *ndev;
		struct xgs_port *pt;

		ndev = alloc_netdev(sizeof(*pt), fp->name, NET_NAME_PREDICTABLE, ether_setup);
		if (!ndev)
			return -ENOMEM;
		SET_NETDEV_DEV(ndev, sc->dev);
		pt = netdev_priv(ndev);
		pt->sc = sc;
		pt->ndev = ndev;
		pt->fp = fp;
		pt->idx = i;
		pt->link = -1;
		if (sc->use_factory && sc->nport[i].have_factory) {
			memcpy(pt->mac, sc->nport[i].factory, ETH_ALEN);
		} else {
			memcpy(pt->mac, sc->hostmac, ETH_ALEN);
			pt->mac[5] = sc->hostmac[5] + fp->unit;
		}
		eth_hw_addr_set(ndev, pt->mac);
		ndev->netdev_ops = &xgs_netdev_ops;
		ndev->ethtool_ops = &xgs_ethtool_ops;
		ndev->min_mtu = ETH_MIN_MTU;
		ndev->max_mtu = sc->buf_size - GIU_HDR_LEN - ETH_HLEN;
		ndev->mtu = GIU_MTU;
		ndev->priv_flags |= IFF_UNICAST_FLT;
		netif_carrier_off(ndev);

		err = register_netdev(ndev);
		if (err) {
			dev_err(sc->dev, "register_netdev %s failed: %d\n", fp->name, err);
			free_netdev(ndev);
			return err;
		}
		/* after registration, which copies dev_addr into perm_addr */
		if (sc->nport[i].have_factory)
			memcpy(ndev->perm_addr, sc->nport[i].factory, ETH_ALEN);
		spin_lock_bh(&sc->lock);
		sc->port[i] = pt;
		if (fp->tag & 0x8000)
			sc->by_hi[(fp->tag >> 8) & 0x7f] = i;
		else if (fp->tag < 8)
			sc->by_lo[fp->tag] = i;
		spin_unlock_bh(&sc->lock);
	}
	return 0;
}

static void giu_teardown(struct xgs *sc)
{
	u8 pz[8] = { 0 };
	int i, q;
	u32 st;
	bool alive = !sc->lost && xgs_endpoint_alive(sc);

	if (sc->datapath && alive && cfg_rd(sc, AGNIC_CFG_STATUS) != 0xffffffffU) {
		bool saved = sc->stopping;

		WRITE_ONCE(sc->stopping, false);	/* allow the two goodbye commands */
		if (!giu_command(sc, AGNIC_CC_PF_DISABLE, pz, 0))
			giu_command(sc, AGNIC_CC_PF_CLOSE, pz, 0);
		WRITE_ONCE(sc->stopping, saved);
	}

	spin_lock_bh(&sc->lock);
	sc->datapath = false;
	spin_unlock_bh(&sc->lock);

	if (sc->poll_task) {
		kthread_stop(sc->poll_task);
		sc->poll_task = NULL;
	}

	for (i = 0; i < NFRONT; i++) {
		struct xgs_port *pt = sc->port[i];

		if (!pt)
			continue;
		spin_lock_bh(&sc->lock);
		sc->port[i] = NULL;
		spin_unlock_bh(&sc->lock);
		unregister_netdev(pt->ndev);
		free_netdev(pt->ndev);
	}

	st = alive ? cfg_rd(sc, AGNIC_CFG_STATUS) : 0xffffffffU;
	if (st != 0xffffffffU) {
		cfg_wr(sc, AGNIC_CFG_STATUS, st & ~AGNIC_STATUS_HOST_MGMT_READY);
		cfg_wr(sc, AGNIC_CFG_CMD_Q + AGNIC_QI_ADDR, 0);
		cfg_wr(sc, AGNIC_CFG_CMD_Q + AGNIC_QI_ADDR + 4, 0);
		cfg_wr(sc, AGNIC_CFG_CMD_Q + AGNIC_QI_LEN, 0);
		cfg_wr(sc, AGNIC_CFG_NOTIF_Q + AGNIC_QI_ADDR, 0);
		cfg_wr(sc, AGNIC_CFG_NOTIF_Q + AGNIC_QI_ADDR + 4, 0);
		cfg_wr(sc, AGNIC_CFG_NOTIF_Q + AGNIC_QI_LEN, 0);
		giu_free_buffers(sc);
		for (q = 0; q < GIU_NUM_QS; q++) {
			giu_free_ring(sc, &sc->bp[q]);
			giu_free_ring(sc, &sc->rx[q]);
			giu_free_ring(sc, &sc->tx[q]);
		}
		giu_free_ring(sc, &sc->notif);
		giu_free_ring(sc, &sc->cmd);
	} else if (sc->giu_up) {
		dev_warn(sc->dev, "giu: endpoint gone before withdrawal - leaking DMA rings on purpose\n");
	}
	dev_info(sc->dev, "giu: %llu commands, %llu answers, %llu notifications (%llu keep-alive), %llu late, %llu unmatched\n",
		 sc->commands, sc->answers, sc->notifications, sc->keepalives, sc->late, sc->drops);
	sc->giu_up = false;
}

/*
 * The NPU advertises 00:01:02:03:04:05 - a firmware constant, identical on every unit - so
 * the port addresses are derived from this appliance's own DMI identity instead:
 * 02:<crc32 of the first usable identifier, 24 bits>:00:<port unit>. Locally administered.
 */
static bool dmi_usable(const char *s)
{
	static const char * const junk[] = { "none", "default string", "not specified",
		"system serial number", "to be filled", "o.e.m.", "0123456789", "unknown", "n/a",
		"03000200-0400-0500-0006-000700080009", "00000000-0000-0000-0000-000000000000",
		"ffffffff-ffff-ffff-ffff-ffffffffffff" };
	int i;

	if (!s || strlen(s) < 4)
		return false;
	for (i = 0; i < ARRAY_SIZE(junk); i++)
		if (!strncasecmp(s, junk[i], strlen(junk[i])))
			return false;
	return true;
}

static void xgs_derive_mac(struct xgs *sc)
{
	static const int fields[] = { DMI_PRODUCT_SERIAL, DMI_BOARD_SERIAL, DMI_PRODUCT_UUID };
	const char *src = NULL;
	u32 crc;
	int i;

	for (i = 0; i < ARRAY_SIZE(fields) && !src; i++)
		if (dmi_usable(dmi_get_system_info(fields[i])))
			src = dmi_get_system_info(fields[i]);
	if (src) {
		crc = crc32_le(~0U, src, strlen(src));
		sc->hostmac[0] = 0x02;
		sc->hostmac[1] = crc >> 16;
		sc->hostmac[2] = crc >> 8;
		sc->hostmac[3] = crc;
		sc->hostmac[4] = 0;
		sc->hostmac[5] = 0;
		dev_info(sc->dev, "port MAC base %pM (from DMI)\n", sc->hostmac);
	} else {
		memcpy(sc->hostmac, sc->mac, ETH_ALEN);
		sc->hostmac[0] = (sc->hostmac[0] | 0x02) & ~0x01;
		dev_warn(sc->dev, "no usable DMI identity - port MACs %pM+n are NOT unique to this unit\n",
			 sc->hostmac);
	}
}

static int giu_attach(struct xgs *sc)
{
	u32 st0, both, duse, lo, hi;
	int err, i, q;

	for (i = 0; i < 16; i++)
		sc->by_hi[i] = -1;
	for (i = 0; i < 8; i++)
		sc->by_lo[i] = -1;
	for (i = 0; i < GIU_NUM_QS; i++) {
		sc->tx[i].prod_idx = sc->tx[i].cons_idx = -1;
		sc->rx[i].prod_idx = sc->rx[i].cons_idx = -1;
		sc->bp[i].prod_idx = sc->bp[i].cons_idx = -1;
	}
	sc->cmd.prod_idx = sc->cmd.cons_idx = -1;
	sc->notif.prod_idx = sc->notif.cons_idx = -1;
	sc->giu_up = true;

	err = giu_wait_status(sc, AGNIC_STATUS_DEV_READY, 10000, "DEV_READY");
	if (err)
		return err;

	st0 = cfg_rd(sc, AGNIC_CFG_STATUS);
	both = AGNIC_STATUS_HOST_MGMT_READY | AGNIC_STATUS_DEV_MGMT_READY;
	if (st0 & both) {
		cfg_wr(sc, AGNIC_CFG_STATUS, st0 & ~both);
		dev_info(sc->dev, "giu: retracted a previous session's handshake (status was %#x)\n", st0);
	}

	lo = cfg_rd(sc, AGNIC_CFG_MAC_ADDR);
	hi = cfg_rd(sc, AGNIC_CFG_MAC_ADDR + 4);
	sc->mac[0] = lo; sc->mac[1] = lo >> 8; sc->mac[2] = lo >> 16; sc->mac[3] = lo >> 24;
	sc->mac[4] = hi; sc->mac[5] = hi >> 8;

	duse = cfg_rd(sc, AGNIC_CFG_DEV_USE_SIZE);
	sc->idx_count = AGNIC_MAX_QUEUES;
	if (duse < AGNIC_CFG_SIZE || duse > sc->giu_size || sc->idx_count * 4 > sc->giu_size - duse ||
	    (duse & 3)) {
		dev_err(sc->dev, "giu: dev_use_size %#x does not fit the window - refusing\n", duse);
		return -ERANGE;
	}
	sc->idx_base = duse;
	dev_info(sc->dev, "giu: device ready, mac %pM, index array at +%#x\n", sc->mac, duse);

	giu_init_slots(sc);
	err = giu_alloc_ring(sc, &sc->cmd, GIU_CMD_Q_LEN, AGNIC_CMD_DESC_SIZE, "command");
	if (!err)
		err = giu_alloc_ring(sc, &sc->notif, GIU_NOTIF_Q_LEN, AGNIC_CMD_DESC_SIZE, "notification");
	if (err)
		return err;

	giu_publish_ring(sc, AGNIC_CFG_CMD_Q, &sc->cmd);
	giu_publish_ring(sc, AGNIC_CFG_NOTIF_Q, &sc->notif);
	wmb();
	cfg_wr(sc, AGNIC_CFG_STATUS, cfg_rd(sc, AGNIC_CFG_STATUS) | AGNIC_STATUS_HOST_MGMT_READY);

	err = giu_wait_status(sc, AGNIC_STATUS_DEV_MGMT_READY, 4000, "DEV_MGMT_READY");
	if (err) {
		dev_err(sc->dev, "giu: the NPU answers HOST_MGMT_READY once per boot - pulse its reset and reload\n");
		return err;
	}

	sc->poll_task = kthread_run(giu_poll_thread, sc, "xgsnpu-poll");
	if (IS_ERR(sc->poll_task)) {
		err = PTR_ERR(sc->poll_task);
		sc->poll_task = NULL;
		return err;
	}

	err = giu_command(sc, AGNIC_CC_PF_MGMT_ECHO, NULL, 0);
	if (err) {
		dev_err(sc->dev, "giu: MGMT_ECHO did not come back (%d)\n", err);
		return err;
	}
	dev_info(sc->dev, "giu: MGMT_ECHO answered - the command channel is up\n");

	giu_capabilities(sc);

	for (q = 0; q < GIU_NUM_QS && !err; q++) {
		err = giu_alloc_ring(sc, &sc->tx[q], GIU_DATA_Q_LEN, AGNIC_TXD_SIZE, "transmit");
		if (!err)
			err = giu_alloc_ring(sc, &sc->rx[q], GIU_DATA_Q_LEN, AGNIC_RXD_SIZE, "receive");
		if (!err)
			err = giu_alloc_ring(sc, &sc->bp[q], GIU_DATA_Q_LEN, AGNIC_BPD_SIZE, "buffer pool");
	}
	if (!err)
		err = giu_alloc_bufs(sc, &sc->buf, &sc->nbuf, GIU_NUM_QS * GIU_DATA_Q_LEN);
	if (!err)
		err = giu_alloc_bufs(sc, &sc->txbuf, &sc->ntxbuf, GIU_DATA_Q_LEN);
	if (err) {
		dev_err(sc->dev, "giu: cannot allocate datapath memory (%d)\n", err);
		return err;
	}

	xgs_derive_mac(sc);

	for (q = 0; q < GIU_NUM_QS; q++)
		for (i = 0; i < sc->rx[q].len; i++)
			put_unaligned_le64(AGNIC_COOKIE_DRIVER_WATERMARK,
					   desc_at(&sc->rx[q], i) + AGNIC_RXD_COOKIE);

	return giu_bringup(sc);
}

/* ------------------------------------------------------------------------------------------ */
/* NetAgent                                                                                    */

static inline u32 nwa_rd(struct xgs *sc, u32 o) { return ioread32(sc->bar0 + sc->nwa_off + o); }
static inline void nwa_wr(struct xgs *sc, u32 o, u32 v) { iowrite32(v, sc->bar0 + sc->nwa_off + o); }

static int nwa_wait(struct xgs *sc, u32 off, u32 want, int ms)
{
	int i;

	for (i = 0; i < ms / 10; i++) {
		u32 v = nwa_rd(sc, off);

		if (v == want)
			return 0;
		if (v == 0xffffffffU)
			return -ENXIO;
		if (READ_ONCE(sc->stopping))
			return -EINTR;
		msleep(10);
	}
	return -ETIMEDOUT;
}

/* one mailbox transaction; nwa_mutex held */
static int nwa_xfer(struct xgs *sc, const u32 *req, int nreq, u32 *reply, int nreply)
{
	int err, i, reqlen = nreq * 4;
	u32 rb, marker, status;

	if (nreq <= 0 || reqlen > (int)sc->nwa_max_req)
		return -EINVAL;
	err = nwa_wait(sc, NWA_STATUS, NWA_STATUS_IDLE, NWA_IDLE_WAIT_MS);
	if (err) {
		sc->nwa_timeouts++;
		return err;
	}
	for (i = 0; i < nreq; i++)
		nwa_wr(sc, sc->nwa_body + i * 4, req[i]);
	nwa_wr(sc, NWA_REQ_LEN, reqlen);
	wmb();
	nwa_wr(sc, NWA_TURN, NWA_TURN_REQUEST);
	sc->nwa_commands++;

	err = nwa_wait(sc, NWA_STATUS, NWA_STATUS_REPLY, NWA_REPLY_WAIT_MS);
	if (err) {
		sc->nwa_timeouts++;
		return err;
	}
	rmb();
	rb = sc->nwa_body + reqlen;
	marker = nwa_rd(sc, rb + NWA_RP_MARKER);
	status = nwa_rd(sc, rb + NWA_RP_STATUS);

	if (reply) {
		int rlen = (int)nwa_rd(sc, NWA_REPLY_LEN);
		int plen = rlen > NWA_RP_PAYLOAD ? rlen - NWA_RP_PAYLOAD : 0;
		int want = (plen + 3) / 4, tail = plen & 3, have = min(want, nreply);

		for (i = 0; i < have; i++)
			reply[i] = nwa_rd(sc, rb + NWA_RP_PAYLOAD + 4 * i);
		if (tail && have == want && have > 0)
			reply[have - 1] &= (1U << (8 * tail)) - 1;
		for (; i < nreply; i++)
			reply[i] = 0;
	}
	nwa_wr(sc, NWA_TURN, NWA_TURN_ACK);

	if (marker != NWA_RP_MARKER_VALUE) {
		dev_warn(sc->dev, "nwa: reply marker %#x, expected %#x\n", marker, NWA_RP_MARKER_VALUE);
		sc->nwa_failures++;
		return -EBADMSG;
	}
	if (status) {
		sc->nwa_failures++;
		return -EIO;
	}
	return 0;
}

static int nwa_command(struct xgs *sc, u32 op, u32 sub, u32 port, const u32 *pl, int npl,
		       u32 *reply, int nreply)
{
	u32 req[NWA_REQ_SIZE / 4] = { 0 };
	int i;

	req[0] = op;
	req[1] = sub;
	req[2] = port;
	for (i = 0; i < npl && 4 + i < NWA_REQ_SIZE / 4; i++)
		req[4 + i] = pl[i];
	return nwa_xfer(sc, req, NWA_REQ_SIZE / 4, reply, nreply);
}

static int nwa_set(struct xgs *sc, int n, u32 sub, u32 v)
{
	return nwa_command(sc, NWA_OP_SET, sub, front_ports[n].tag, &v, 1, NULL, 0);
}

static int nwa_get(struct xgs *sc, int n, u32 sub, u32 *out)
{
	u32 z = 0;

	return nwa_command(sc, NWA_OP_GET, sub, front_ports[n].tag, &z, 1, out, 1);
}

static int nwa_set_mac(struct xgs *sc, int n)
{
	const u8 *mac;
	u32 pl[2];

	if (!sc->port[n])
		return -ENXIO;
	mac = sc->port[n]->mac;
	pl[0] = mac[0] | (mac[1] << 8) | (mac[2] << 16) | ((u32)mac[3] << 24);
	pl[1] = mac[4] | (mac[5] << 8);
	return nwa_command(sc, NWA_OP_SET, NWA_SUB_MAC, front_ports[n].tag, pl, 2, NULL, 0);
}

/*
 * Read the port's address from NetAgent before this driver sets its own: on a freshly reset
 * NPU that is the address Sophos assigned the port. Read before the netdevs are created, so
 * they can be registered with it (mac_src=factory) and always report it via ethtool -P.
 */
static void nwa_read_factory_mac(struct xgs *sc, int n)
{
	struct nwa_port *p = &sc->nport[n];
	u32 z = 0, r[2] = { 0, 0 };
	u8 *m = p->factory;

	if (nwa_command(sc, NWA_OP_GET, NWA_SUB_MAC, front_ports[n].tag, &z, 1, r, 2))
		return;
	m[0] = r[0]; m[1] = r[0] >> 8; m[2] = r[0] >> 16; m[3] = r[0] >> 24;
	m[4] = r[1]; m[5] = r[1] >> 8;
	if (!is_valid_ether_addr(m))
		return;
	p->have_factory = true;
}

static void nwa_bring_up(struct xgs *sc)
{
	int n, up = 0, addressed = 0;
	u32 v;

	for (n = 0; n < NFRONT; n++) {
		struct nwa_port *p = &sc->nport[n];

		p->link = -1;
		p->media = -1;
		p->promisc = 0;
		if (nwa_set(sc, n, NWA_SUB_STATE, 1)) {
			dev_warn(sc->dev, "nwa: %s refused to come up\n", front_ports[n].name);
			continue;
		}
		p->up = 1;
		up++;
		if (nwa_set_mac(sc, n))
			dev_warn(sc->dev, "nwa: %s would not take its address - broadcast only\n",
				 front_ports[n].name);
		else
			addressed++;
		if (!nwa_get(sc, n, NWA_SUB_MEDIA, &v))
			p->media = v;
	}
	dev_info(sc->dev, "nwa: %d of %d ports up, %d told their own address\n", up, NFRONT, addressed);
}

static void nwa_link_step(struct xgs *sc)
{
	int n = sc->nwa_sweep;
	struct nwa_port *p;
	struct xgs_port *pt;
	u32 v;

	if (n < 0 || n >= NFRONT)
		n = 0;
	p = &sc->nport[n];
	pt = sc->port[n];

	if (p->up && !nwa_get(sc, n, NWA_SUB_STATE, &v)) {
		int link = v != 0;

		if (link != p->link) {
			int speed = 0;

			p->link = link;
			if (link && !nwa_get(sc, n, NWA_SUB_SPEED, &v))
				speed = v;
			dev_info(sc->dev, "nwa: %s (0x%04x, %s) carrier %s%s\n", front_ports[n].name,
				 front_ports[n].tag, p->media == 3 ? "fibre" : "copper",
				 link ? "up" : "down", "");
			if (pt) {
				pt->link = link;
				pt->speed = speed;
				if (link)
					netif_carrier_on(pt->ndev);
				else
					netif_carrier_off(pt->ndev);
			}
		}
	}

	/* reconcile the switch's catch-all with what the interface wants (bridges need it) */
	if (p->up && pt) {
		int want = !!(pt->ndev->flags & IFF_PROMISC);

		if (want != p->promisc && !nwa_set(sc, n, NWA_SUB_PROMISC, want)) {
			p->promisc = want;
			dev_info(sc->dev, "nwa: %s catch-all %s\n", front_ports[n].name,
				 want ? "opened" : "closed");
		}
	}
	sc->nwa_sweep = (n + 1) % NFRONT;
}

/*
 * Wait for NetAgent to publish its mailbox (about ten seconds after the handshake) and
 * validate it. nwa_mutex held. Returns 0 when the mailbox is usable.
 */
static int nwa_wait_mailbox(struct xgs *sc)
{
	u32 cookie = 0, body, maxreq;

	for (sc->nwa_tries = 0; sc->nwa_tries < NWA_READY_TRIES; sc->nwa_tries++) {
		cookie = nwa_rd(sc, NWA_COOKIE);
		if (cookie == NWA_COOKIE_VALUE)
			break;
		if (sc->nwa_tries == 0)
			dev_info(sc->dev, "nwa: waiting for the network agent to publish its window\n");
		if (!xgs_sleep(sc, 500))
			return -EINTR;
	}
	if (cookie != NWA_COOKIE_VALUE) {
		dev_err(sc->dev, "nwa: the network agent never appeared (cookie %#x) - ports stay down\n",
			cookie);
		return -ETIMEDOUT;
	}
	body = nwa_rd(sc, NWA_BODY_OFF);
	if (body != NWA_BODY_EXPECTED) {
		dev_err(sc->dev, "nwa: mailbox body offset %#x, this driver speaks %#x - refusing\n",
			body, NWA_BODY_EXPECTED);
		return -EPROTO;
	}
	maxreq = nwa_rd(sc, NWA_MAX_REQ);
	if (maxreq < NWA_REQ_SIZE || maxreq > sc->nwa_size) {
		dev_err(sc->dev, "nwa: maximum request %u does not fit the window - refusing\n", maxreq);
		return -EPROTO;
	}
	sc->nwa_body = body;
	sc->nwa_max_req = maxreq;
	dev_info(sc->dev, "nwa: mailbox ready after %d ms, requests up to %u bytes\n",
		 sc->nwa_tries * 500, maxreq);
	return 0;
}

/* runs only once the mailbox is known: bring the ports up, then poll link state */
static void nwa_work_fn(struct work_struct *w)
{
	struct xgs *sc = container_of(to_delayed_work(w), struct xgs, nwa_work);

	if (READ_ONCE(sc->stopping))
		return;
	mutex_lock(&sc->nwa_mutex);
	if (!sc->nwa_ready) {
		nwa_bring_up(sc);
		sc->nwa_ready = true;
		sc->nwa_sweep = 0;
	} else {
		nwa_link_step(sc);
	}
	mutex_unlock(&sc->nwa_mutex);
	if (!READ_ONCE(sc->stopping))
		queue_delayed_work(sc->wq, &sc->nwa_work, msecs_to_jiffies(NWA_LINK_TICK_MS));
}

static void nwa_teardown(struct xgs *sc)
{
	int n;

	cancel_delayed_work_sync(&sc->nwa_work);
	mutex_lock(&sc->nwa_mutex);
	if (sc->nwa_ready && !sc->lost && xgs_endpoint_alive(sc) &&
	    nwa_rd(sc, NWA_COOKIE) == NWA_COOKIE_VALUE) {
		bool saved = sc->stopping;

		WRITE_ONCE(sc->stopping, false);
		for (n = 0; n < NFRONT; n++)
			if (sc->nport[n].up)
				nwa_set(sc, n, NWA_SUB_STATE, 0);
		WRITE_ONCE(sc->stopping, saved);
	}
	sc->nwa_ready = false;
	mutex_unlock(&sc->nwa_mutex);
	dev_info(sc->dev, "nwa: %llu commands, %llu refused, %llu timed out\n",
		 sc->nwa_commands, sc->nwa_failures, sc->nwa_timeouts);
}

/* ------------------------------------------------------------------------------------------ */
/* control-message (rpc) channel                                                               */

static inline u32 rpc_rd(struct xgs *sc, u32 o) { return ioread32(sc->bar2 + sc->rpc_off + o); }
static inline void rpc_wr(struct xgs *sc, u32 o, u32 v) { iowrite32(v, sc->bar2 + sc->rpc_off + o); }

static void rpc_layout_one(struct xgs *sc, u32 r, u32 desc, u32 count, u32 cfg)
{
	u32 i;

	for (i = 0; i < count * 16; i += 4)
		rpc_wr(sc, desc + i, 0);
	rpc_wr(sc, r + RPC_RING_POSTED, 0);
	rpc_wr(sc, r + RPC_RING_POSTED + 4, 0);
	rpc_wr(sc, r + RPC_RING_DONE, 0);
	rpc_wr(sc, r + RPC_RING_DONE + 4, 0);
	rpc_wr(sc, r + RPC_RING_OFFSET, desc);
	rpc_wr(sc, r + RPC_RING_DESC_OFFSET, desc);
	rpc_wr(sc, r + RPC_RING_DESC_COUNT, count);
	rpc_wr(sc, r + RPC_RING_CFG, cfg);
}

static int rpc_wait_reconfig(struct xgs *sc)
{
	int i;

	for (i = 0; i < 500; i++) {
		if (rpc_rd(sc, RPC_ST_CFG_REVISION) >> 24)
			return 0;
		if (!xgs_sleep(sc, 10))
			return -EINTR;
	}
	return -ETIMEDOUT;
}

static int rpc_open(struct xgs *sc)
{
	u32 o, st;
	int err;

	for (o = 0; o < RPC_STATE_SIZE; o += 4)
		rpc_wr(sc, o, 0);
	/* low ring: ring 0, facility 3, doorbell 0, not shared */
	rpc_layout_one(sc, RPC_ST_RING_LO, RPC_DESC_OFF, RPC_DESC_COUNT, 0 | (3 << 8) | (0 << 16) | (0 << 24));
	/* high ring 0: ring 1, same facility and doorbell, shared - keeps the target's tasklet sane */
	rpc_layout_one(sc, RPC_ST_RINGS, RPC_HI_DESC_OFF, RPC_HI_DESC_COUNT, 1 | (3 << 8) | (0 << 16) | (1 << 24));
	wmb();

	writeq(0, sc->bar2 + sc->rpc_off + RPC_ST_CFG_MAGIC);
	wmb();
	xgs_ring_dbell(sc, 0);
	err = rpc_wait_reconfig(sc);
	if (err) {
		dev_err(sc->dev, "rpc: target did not acknowledge the empty configuration (state %#x)\n",
			rpc_rd(sc, RPC_ST_CFG_REVISION));
		return err;
	}

	writeq((u64)RPC_STATE_CFG_MAGIC | ((u64)RPC_CFG_REVISION << 32) | ((u64)1 << 48),
	       sc->bar2 + sc->rpc_off + RPC_ST_CFG_MAGIC);
	wmb();
	xgs_ring_dbell(sc, 0);
	err = rpc_wait_reconfig(sc);
	if (err) {
		dev_err(sc->dev, "rpc: target did not take the configuration (magic %#x state %#x)\n",
			rpc_rd(sc, RPC_ST_CFG_MAGIC), rpc_rd(sc, RPC_ST_CFG_REVISION));
		return err;
	}
	if (rpc_rd(sc, RPC_ST_CFG_MAGIC) != RPC_STATE_CFG_MAGIC) {
		dev_err(sc->dev, "rpc: magic did not stick (%#x)\n", rpc_rd(sc, RPC_ST_CFG_MAGIC));
		return -EIO;
	}
	sc->rpc_opened = true;
	sc->rpc_stalled = false;
	st = rpc_rd(sc, RPC_ST_CFG_REVISION);
	dev_info(sc->dev, "rpc: channel open - revision %u, %u high ring\n", st & 0xffff, (st >> 16) & 0xff);
	return 0;
}

/* returns payload length (>= 0) or -errno; rpc_mutex held */
static int rpc_command(struct xgs *sc, u8 cmd, const void *payload, int plen, void *resp, int rlen,
		       int *rcout)
{
	u8 *b = sc->rpc_cmd;
	u64 want, done = 0;
	u32 d;
	u16 rc, rpl;
	int i;

	if (!sc->rpc_opened)
		return -ENXIO;
	if (sc->rpc_stalled)
		return -ESHUTDOWN;
	if (plen < 0 || plen + RPC_CMD_PAYLOAD > RPC_DATA_MAX_SIZE)
		return -EINVAL;

	memset(b, 0, RPC_DATA_MAX_SIZE);
	put_unaligned_le16(RPC_DATA_MAX_SIZE / 2, b + RPC_CMD_RESP_BUFF_SZ);
	b[RPC_CMD_CMD] = cmd;
	if (plen)
		memcpy(b + RPC_CMD_PAYLOAD, payload, plen);

	want = sc->rpc_posted + 1;
	d = RPC_DESC_OFF + (u32)((want - 1) & (RPC_DESC_COUNT - 1)) * 16;
	rpc_wr(sc, d + RPC_BD_DMA_ADDR, lower_32_bits(sc->rpc_cmd_pa));
	rpc_wr(sc, d + RPC_BD_DMA_ADDR + 4, upper_32_bits(sc->rpc_cmd_pa));
	rpc_wr(sc, d + RPC_BD_PAYLOAD_LEN, (u32)plen | ((u32)RPC_DESC_NO_AGG_DMA << 16));
	rpc_wr(sc, d + 12, 0);
	wmb();
	rpc_wr(sc, RPC_ST_RING_LO + RPC_RING_POSTED, lower_32_bits(want));
	rpc_wr(sc, RPC_ST_RING_LO + RPC_RING_POSTED + 4, upper_32_bits(want));
	wmb();
	xgs_ring_dbell(sc, 0);

	for (i = 0; i < 300; i++) {
		done = (u64)rpc_rd(sc, RPC_ST_RING_LO + RPC_RING_DONE) |
		       ((u64)rpc_rd(sc, RPC_ST_RING_LO + RPC_RING_DONE + 4) << 32);
		if (done >= want)
			break;
		if (!xgs_sleep(sc, 10))
			return -EINTR;
	}
	if (done < want) {
		sc->rpc_stalled = true;
		dev_err(sc->dev, "rpc: command %u unanswered (posted %llu done %llu) - channel stalled\n",
			cmd, want, done);
		return -ETIMEDOUT;
	}
	sc->rpc_posted = want;
	rmb();
	if (!b[RPC_RESP_DESC_DONE]) {
		dev_warn(sc->dev, "rpc: command %u completed without an answer\n", cmd);
		return -EIO;
	}
	rc = get_unaligned_le16(b + RPC_RESP_RC);
	rpl = get_unaligned_le16(b + RPC_RESP_PAYLOAD_LEN);
	if (rcout)
		*rcout = rc;
	if (resp && rlen > 0) {
		if (rpl > rlen)
			rpl = rlen;
		if (RPC_RESP_PAYLOAD + rpl <= RPC_DATA_MAX_SIZE)
			memcpy(resp, b + RPC_RESP_PAYLOAD, rpl);
	}
	return rpl;
}

/* program every front port's LIF + pport binding, read back, count verified */
static int rpc_program_front_ports(struct xgs *sc)
{
	u8 pl[RPC_LIF_REQ_SIZE], back[12];
	int i, rc, n, pass, verified = 0;

	if (!sc->rpc_opened || sc->rpc_stalled)
		return 0;

	for (i = 0; i < NFRONT; i++) {
		const struct front_port *fp = &front_ports[i];
		u32 idx = (u32)fp->iface_id << RPC_LIF_IFACE_SHIFT;
		const u8 *mac;

		if (!sc->port[i])
			continue;
		mac = sc->port[i]->mac;

		for (pass = 0; pass < 2; pass++) {
			memset(pl, 0, sizeof(pl));
			put_unaligned_le32(idx, pl + RPC_TBL_S_INDEX);
			put_unaligned_le16(1, pl + RPC_TBL_NUM_ENTRIES);
			memset(back, 0, sizeof(back));
			rc = 0;
			n = rpc_command(sc, RPC_CMD_LO_LIF_READ, pl, RPC_TBL_REQ_SIZE, back, sizeof(back), &rc);
			if (n == (int)sizeof(back) && rc == 0 && !memcmp(back, mac, ETH_ALEN) &&
			    get_unaligned_le16(back + 8) == (RPC_LIF_FWD_L2 | RPC_LIF_OFFLOAD_DISABLED)) {
				verified++;
				break;
			}
			if (pass || n == -ESHUTDOWN || n == -EINTR)
				break;

			memset(pl, 0, sizeof(pl));
			put_unaligned_le32(idx, pl + RPC_LIF_INDEX);
			memcpy(pl + RPC_LIF_MAC, mac, ETH_ALEN);
			put_unaligned_le16(RPC_LIF_MTU_VAL, pl + RPC_LIF_MTU);
			put_unaligned_le16(RPC_LIF_FWD_L2 | RPC_LIF_OFFLOAD_DISABLED, pl + RPC_LIF_FLAGS);
			put_unaligned_le16(RPC_LIF_CREATE, pl + RPC_LIF_UPDATE_MASK);
			if (rpc_command(sc, RPC_CMD_LIF_ADD_UPDATE, pl, RPC_LIF_REQ_SIZE, NULL, 0, &rc) < 0)
				break;

			memset(pl, 0, RPC_PPORT_REQ_SIZE);
			pl[RPC_PPORT_IFACE] = fp->iface_id;
			put_unaligned_le16(fp->tag, pl + RPC_PPORT_TAG);
			rpc_command(sc, RPC_CMD_PPORT_UPDATE, pl, RPC_PPORT_REQ_SIZE, NULL, 0, &rc);
		}
	}
	return verified;
}

static void rpc_work_fn(struct work_struct *w)
{
	struct xgs *sc = container_of(to_delayed_work(w), struct xgs, rpc_work);
	bool done, stop = false;

	if (READ_ONCE(sc->stopping))
		return;
	mutex_lock(&sc->rpc_mutex);
	if (!sc->rpc_opened) {
		if (rpc_open(sc)) {
			mutex_unlock(&sc->rpc_mutex);
			return;
		}
	}
	sc->rpc_verified = rpc_program_front_ports(sc);
	sc->rpc_tries++;
	done = sc->rpc_verified == NFRONT;
	if (done && !sc->rpc_announced) {
		dev_info(sc->dev, "rpc: all %d front ports have an interface and a binding, read back and confirmed\n",
			 NFRONT);
		sc->rpc_announced = true;
	} else if (!done) {
		if (sc->rpc_announced)
			dev_info(sc->dev, "rpc: the NPU fastpath cleared its interface table - programming again\n");
		sc->rpc_announced = false;
	}
	if (!done && sc->rpc_tries >= RPC_PROG_TRIES) {
		dev_err(sc->dev, "rpc: gave up - only %d of %d front ports read back correctly\n",
			sc->rpc_verified, NFRONT);
		stop = true;
	}
	/* keep checking a while after success: the fastpath wipes the table ~13 s after start */
	if (done && sc->rpc_tries >= RPC_PROG_TRIES / 2)
		stop = true;
	if (sc->rpc_stalled)
		stop = true;
	mutex_unlock(&sc->rpc_mutex);
	if (!stop && !READ_ONCE(sc->stopping))
		queue_delayed_work(sc->wq, &sc->rpc_work, msecs_to_jiffies(RPC_PROG_RETRY_MS));
}

static void rpc_teardown(struct xgs *sc)
{
	cancel_delayed_work_sync(&sc->rpc_work);
	mutex_lock(&sc->rpc_mutex);
	if (sc->rpc_opened && !sc->lost && xgs_endpoint_alive(sc))
		rpc_wr(sc, RPC_ST_CFG_MAGIC, 0);
	sc->rpc_opened = false;
	mutex_unlock(&sc->rpc_mutex);
}

/* ------------------------------------------------------------------------------------------ */
/* bring-up, probe, remove                                                                     */

static void xgs_bringup(struct work_struct *w)
{
	struct xgs *sc = container_of(w, struct xgs, bringup_work);
	int err;

	if (!xgs_endpoint_alive(sc)) {
		dev_err(sc->dev, "endpoint not answering config space - NPU held in reset? not touching BARs\n");
		return;
	}
	if (ioread32(sc->bar0) == 0xffffffffU && ioread32(sc->bar0 + 4) == 0xffffffffU) {
		dev_err(sc->dev, "BAR0 reads all ones - NPU not running or BARs not restored (remove+rescan)\n");
		return;
	}

	err = xgs_wait_barmap(sc);
	if (err)
		return;

	err = xgs_setup_msix(sc);
	if (err)
		return;

	if (!(stages & 1))
		return;
	err = xgs_handshake(sc);
	if (err)
		return;
	queue_delayed_work(sc->wq, &sc->heartbeat_work, HZ);

	if (!(stages & 2) || !sc->giu_size)
		return;
	err = giu_attach(sc);
	if (err) {
		dev_err(sc->dev, "giu: datapath did not come up (%d)\n", err);
		return;
	}

	/* the factory addresses come from NetAgent, so it has to be up before the netdevs exist */
	sc->use_factory = !strcmp(mac_src, "factory");
	if ((stages & 4) && sc->nwa_size) {
		int n, got = 0;

		mutex_lock(&sc->nwa_mutex);
		sc->nwa_mailbox = !nwa_wait_mailbox(sc);
		if (sc->nwa_mailbox)
			for (n = 0; n < NFRONT; n++) {
				nwa_read_factory_mac(sc, n);
				got += sc->nport[n].have_factory;
			}
		mutex_unlock(&sc->nwa_mutex);
		if (sc->nwa_mailbox)
			dev_info(sc->dev, "nwa: %d of %d factory MACs read (first %pM)\n", got, NFRONT,
				 sc->nport[0].factory);
	}
	if (sc->use_factory)
		dev_info(sc->dev, "ports use their factory MACs where the NPU reported one\n");

	err = giu_register_netdevs(sc);
	if (err) {
		dev_err(sc->dev, "giu: could not register the port interfaces (%d)\n", err);
		return;
	}
	dev_info(sc->dev, "giu: datapath enabled - %d interfaces, %d descriptors each way\n",
		 NFRONT, GIU_DATA_Q_LEN);

	if (sc->nwa_mailbox)
		queue_delayed_work(sc->wq, &sc->nwa_work, 0);
	if ((stages & 8) && sc->rpc_size) {
		sc->rpc_cmd = dma_alloc_coherent(sc->dev, RPC_DATA_MAX_SIZE, &sc->rpc_cmd_pa, GFP_KERNEL);
		if (sc->rpc_cmd)
			queue_delayed_work(sc->wq, &sc->rpc_work, 0);
		else
			dev_err(sc->dev, "rpc: no DMA buffer for commands\n");
	}
}

static int xgs_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	struct xgs *sc;
	int err;

	sc = devm_kzalloc(&pdev->dev, sizeof(*sc), GFP_KERNEL);
	if (!sc)
		return -ENOMEM;
	sc->pdev = pdev;
	sc->dev = &pdev->dev;
	pci_set_drvdata(pdev, sc);
	spin_lock_init(&sc->lock);
	mutex_init(&sc->cmd_mutex);
	mutex_init(&sc->nwa_mutex);
	mutex_init(&sc->rpc_mutex);
	init_waitqueue_head(&sc->cmd_wq);
	INIT_WORK(&sc->bringup_work, xgs_bringup);
	INIT_DELAYED_WORK(&sc->heartbeat_work, xgs_heartbeat);
	INIT_DELAYED_WORK(&sc->nwa_work, nwa_work_fn);
	INIT_DELAYED_WORK(&sc->rpc_work, rpc_work_fn);

	if (!xgs_endpoint_alive(sc))
		return dev_err_probe(&pdev->dev, -ENXIO, "endpoint not answering config space\n");

	err = pcim_enable_device(pdev);
	if (err)
		return err;
	err = pcim_request_all_regions(pdev, DRV_NAME);
	if (err)
		return err;
	sc->bar0 = pcim_iomap(pdev, 0, 0);
	sc->bar2 = pcim_iomap(pdev, 2, 0);
	sc->bar4 = pcim_iomap(pdev, 4, 0);
	if (!sc->bar0 || !sc->bar2)
		return dev_err_probe(&pdev->dev, -ENOMEM, "cannot map BAR0/BAR2\n");
	sc->bar0_len = pci_resource_len(pdev, 0);
	sc->bar2_len = pci_resource_len(pdev, 2);
	sc->bar4_len = sc->bar4 ? pci_resource_len(pdev, 4) : 0;
	if (sc->bar2_len < NPU_BARMAP_TAIL)
		return dev_err_probe(&pdev->dev, -ENXIO, "BAR2 too small\n");
	sc->window = sc->bar2_len - NPU_BARMAP_TAIL;

	err = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(NPU_DMA_BITS));
	if (err)
		return dev_err_probe(&pdev->dev, err, "no usable %d-bit DMA\n", NPU_DMA_BITS);
	pci_set_master(pdev);

	sc->wq = alloc_ordered_workqueue(DRV_NAME, 0);
	if (!sc->wq) {
		pci_clear_master(pdev);
		return -ENOMEM;
	}
	dev_info(&pdev->dev, "BAR0 %pa BAR2 %pa BAR4 %pa, facility window at BAR2+%#x\n",
		 &sc->bar0_len, &sc->bar2_len, &sc->bar4_len, sc->window);
	queue_work(sc->wq, &sc->bringup_work);
	return 0;
}

static void xgs_remove(struct pci_dev *pdev)
{
	struct xgs *sc = pci_get_drvdata(pdev);

	WRITE_ONCE(sc->stopping, true);
	wake_up(&sc->cmd_wq);
	cancel_work_sync(&sc->bringup_work);

	rpc_teardown(sc);
	nwa_teardown(sc);
	giu_teardown(sc);
	if (sc->rpc_cmd)
		dma_free_coherent(sc->dev, RPC_DATA_MAX_SIZE, sc->rpc_cmd, sc->rpc_cmd_pa);
	sc->rpc_cmd = NULL;

	cancel_delayed_work_sync(&sc->heartbeat_work);
	if (!sc->lost && xgs_endpoint_alive(sc))
		xgs_withdraw_handshake(sc);
	xgs_free_msix(sc);
	destroy_workqueue(sc->wq);
	pci_clear_master(pdev);
}

/* reboot: never block; just tell the NPU we are gone and stop its DMA */
static void xgs_shutdown(struct pci_dev *pdev)
{
	struct xgs *sc = pci_get_drvdata(pdev);

	if (!sc)
		return;
	WRITE_ONCE(sc->stopping, true);
	if (!sc->lost && xgs_endpoint_alive(sc))
		xgs_withdraw_handshake(sc);
	pci_clear_master(pdev);
}

static const struct pci_device_id xgs_ids[] = {
	{ PCI_DEVICE(0x11ab, 0x7080) },
	{ }
};
MODULE_DEVICE_TABLE(pci, xgs_ids);

static struct pci_driver xgs_driver = {
	.name		= DRV_NAME,
	.id_table	= xgs_ids,
	.probe		= xgs_probe,
	.remove		= xgs_remove,
	.shutdown	= xgs_shutdown,
};
module_pci_driver(xgs_driver);

MODULE_DESCRIPTION("Sophos XGS 126/136 front ports via the Marvell CN913x NPU (stock firmware)");
MODULE_LICENSE("Dual BSD/GPL");
