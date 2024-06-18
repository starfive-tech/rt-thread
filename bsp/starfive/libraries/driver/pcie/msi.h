

struct msi_msg {
	uint32_t		address_lo;
	uint32_t		address_hi;
	uint32_t		data;
};

struct pci_msi_desc {
	union {
		uint32_t msi_mask;
		uint32_t msix_ctrl;
	};
	struct {
		uint8_t	is_msix		: 1;
		uint8_t	multiple	: 3;
		uint8_t	multi_cap	: 3;
		uint8_t	can_mask	: 1;
		uint8_t	is_64		: 1;
		uint8_t	is_virtual	: 1;
		unsigned default_irq;
	} msi_attrib;
	union {
		uint8_t	mask_pos;
		void  *mask_base;
	};
};

struct msi_desc {
	/* Shared device/bus type independent data */
	unsigned int			irq;
	unsigned int			nvec_used;
	//struct device			*dev;
	struct msi_msg			msg;

	//void (*write_msi_msg)(struct msi_desc *entry, void *data);
	//void *write_msi_msg_data;

	uint16_t			msi_index;
	struct pci_msi_desc	pci;
};

struct udevice;

struct pci_msi_dev {
	struct udevice *dev;
	struct msi_desc desc[4];
	void *msix_base;
	uint8_t current_state;
	uint8_t msix_hwsize;
	uint8_t msix_cap;
	uint8_t msi_cap;
	uint8_t pm_cap;
	uint8_t irq;

	uint32_t pme_support:1;
	uint32_t no_64bit_msi:1;
	uint32_t d3cold_allowed:1;
	uint32_t d1_support:1;
	uint32_t d2_support:1;
	uint32_t pme_poll:1;
	uint32_t imm_ready:1;
	uint32_t msi_enabled:1;
	uint32_t msix_enabled:1;
};

/* Message Signaled Interrupt registers */

#define PCI_MSI_FLAGS		0x02	/* Message Control */
#define  PCI_MSI_FLAGS_ENABLE	0x0001	/* MSI feature enabled */
#define  PCI_MSI_FLAGS_QMASK	0x000e	/* Maximum queue size available */
#define  PCI_MSI_FLAGS_QSIZE	0x0070	/* Message queue size configured */
#define  PCI_MSI_FLAGS_64BIT	0x0080	/* 64-bit addresses allowed */
#define  PCI_MSI_FLAGS_MASKBIT	0x0100	/* Per-vector masking capable */
#define PCI_MSI_RFU		3	/* Rest of capability flags */
#define PCI_MSI_ADDRESS_LO	0x04	/* Lower 32 bits */
#define PCI_MSI_ADDRESS_HI	0x08	/* Upper 32 bits (if PCI_MSI_FLAGS_64BIT set) */
#define PCI_MSI_DATA_32		0x08	/* 16 bits of data for 32-bit devices */
#define PCI_MSI_MASK_32		0x0c	/* Mask bits register for 32-bit devices */
#define PCI_MSI_PENDING_32	0x10	/* Pending intrs for 32-bit devices */
#define PCI_MSI_DATA_64		0x0c	/* 16 bits of data for 64-bit devices */
#define PCI_MSI_MASK_64		0x10	/* Mask bits register for 64-bit devices */
#define PCI_MSI_PENDING_64	0x14	/* Pending intrs for 64-bit devices */

/* MSI-X registers (in MSI-X capability) */
#define PCI_MSIX_FLAGS		2	/* Message Control */
#define  PCI_MSIX_FLAGS_QSIZE	0x07FF	/* Table size */
#define  PCI_MSIX_FLAGS_MASKALL	0x4000	/* Mask all vectors for this function */
#define  PCI_MSIX_FLAGS_ENABLE	0x8000	/* MSI-X enable */
#define PCI_MSIX_TABLE		4	/* Table offset */
#define  PCI_MSIX_TABLE_BIR	0x00000007 /* BAR index */
#define  PCI_MSIX_TABLE_OFFSET	0xfffffff8 /* Offset into specified BAR */
#define PCI_MSIX_PBA		8	/* Pending Bit Array offset */
#define  PCI_MSIX_PBA_BIR	0x00000007 /* BAR index */
#define  PCI_MSIX_PBA_OFFSET	0xfffffff8 /* Offset into specified BAR */
#define PCI_MSIX_FLAGS_BIRMASK	PCI_MSIX_PBA_BIR /* deprecated */
#define PCI_CAP_MSIX_SIZEOF	12	/* size of MSIX registers */

/* MSI-X Table entry format (in memory mapped by a BAR) */
#define PCI_MSIX_ENTRY_SIZE		16
#define PCI_MSIX_ENTRY_LOWER_ADDR	0x0  /* Message Address */
#define PCI_MSIX_ENTRY_UPPER_ADDR	0x4  /* Message Upper Address */
#define PCI_MSIX_ENTRY_DATA		0x8  /* Message Data */
#define PCI_MSIX_ENTRY_VECTOR_CTRL	0xc  /* Vector Control */
#define  PCI_MSIX_ENTRY_CTRL_MASKBIT	0x00000001


static inline void *pci_msix_desc_addr(struct msi_desc *desc)
{
	return desc->pci.mask_base + desc->msi_index * PCI_MSIX_ENTRY_SIZE;
}


void pcie_init_capabilities(struct pci_msi_dev *dev);
int pcie_alloc_irq_vectors(struct pci_msi_dev *dev, unsigned int min_vecs,
				   unsigned int max_vecs, unsigned int msi_irq, unsigned int flags);


