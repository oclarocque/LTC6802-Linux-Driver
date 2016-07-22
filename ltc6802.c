#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/delay.h>
#include <linux/iio/iio.h>

/* LTC6802 Commands */
/* Write Configuration Register Group */
#define LTC6802_CMD_WRCFG		0x01
/* Read Configuration Register Group */
#define LTC6802_CMD_RDCFG		0x02
/* Read Cell Voltage Register Group */
#define LTC6802_CMD_RDCV		0x04
/* Read Flag Register Group */
#define LTC6802_CMD_RDFLG		0x06
/* Read Temperature Register Group */
#define LTC6802_CMD_RDTMP		0x08
/* Start Cell Voltage A/D Conversions and Poll Status */
#define LTC6802_CMD_STCVAD		0x10
/* Start Open-Wire A/D Conversions and Poll Status */
#define LTC6802_CMD_STOWAD		0x20
/* Start Temperature A/D Conversions and Poll Status */
#define LTC6802_CMD_STTMPAD		0x30
/* Poll A/D Converter Status */
#define LTC6802_CMD_PLADC		0x40
/* Poll Interrupt Status */
#define LTC6802_CMD_PLINT		0x50
/* Start Cell Voltage A/D Conversions and Poll Status, w/ Discharge Permitted */
#define LTC6802_CMD_STCVDC		0x60
/* Start Open-Wire A/D Conversions and Poll Status, w/ Discharge Permitted */
#define LTC6802_CMD_STOWDC		0x70

/* LTC6802 Configuration Register Groups */
/* Register 0 */
#define LTC6802_CFGR0_WDT		BIT(7)
#define LTC6802_CFGR0_GPIO2		BIT(6)
#define LTC6802_CFGR0_GPIO1		BIT(5)
#define LTC6802_CFGR0_LVPL		BIT(4)
#define LTC6802_CFGR0_CELL10		BIT(3)
#define LTC6802_CFGR0_CDC_MODE7		7
#define LTC6802_CFGR0_CDC_MODE6		6
#define LTC6802_CFGR0_CDC_MODE5		5
#define LTC6802_CFGR0_CDC_MODE4		4
#define LTC6802_CFGR0_CDC_MODE3		3
#define LTC6802_CFGR0_CDC_MODE2		2
#define LTC6802_CFGR0_CDC_MODE1		1
#define LTC6802_CFGR0_CDC_MODE0		0
/* Register 1 */
#define LTC6802_CFGR1_DCC7		BIT(7)
#define LTC6802_CFGR1_DCC6		BIT(6)
#define LTC6802_CFGR1_DCC5		BIT(5)
#define LTC6802_CFGR1_DCC4		BIT(4)
#define LTC6802_CFGR1_DCC3		BIT(3)
#define LTC6802_CFGR1_DCC2		BIT(2)
#define LTC6802_CFGR1_DCC1		BIT(1)
#define LTC6802_CFGR1_DCC0		BIT(0)
/* Register 2 */
#define LTC6802_CFGR2_MC4I		BIT(7)
#define LTC6802_CFGR2_MC3I		BIT(6)
#define LTC6802_CFGR2_MC2I		BIT(5)
#define LTC6802_CFGR2_MC1I		BIT(4)
#define LTC6802_CFGR2_DCC12		BIT(3)
#define LTC6802_CFGR2_DCC11		BIT(2)
#define LTC6802_CFGR2_DCC10		BIT(1)
#define LTC6802_CFGR2_DCC9		BIT(0)
/* Register 3 */
#define LTC6802_CFGR3_MC12I		BIT(7)
#define LTC6802_CFGR3_MC11I		BIT(6)
#define LTC6802_CFGR3_MC10I		BIT(5)
#define LTC6802_CFGR3_MC9I		BIT(4)
#define LTC6802_CFGR3_MC8I		BIT(3)
#define LTC6802_CFGR3_MC7I		BIT(2)
#define LTC6802_CFGR3_MC6I		BIT(1)
#define LTC6802_CFGR3_MC5I		BIT(0)

#define LTC6802_INPUT_DELTA_MV		6144
#define LTC6802_ADC_RESOLUTION_BIT	12
#define LTC6802_ADDR_CMD_SOF		(1000 << 4)
#define LTC6802_CDC_MASK		0x07
#define LTC6802_CHAN(n)   		(n + 1)

#define LTC6802_ATTR_NAME_TO_ID(name) 	(((int)name[4] - 0x30) * 10 	\
					+ (int)name[5] - 0x30)

#define LTC6802_DEVICE_ATTR(name)	DEVICE_ATTR(name,		\
						    S_IWUSR | S_IRUGO,	\
						    ltc6802_pin_show,	\
						    ltc6802_pin_store);

enum ltc6802_register_group {
	LTC6802_REG_CFG,
	LTC6802_REG_CV,
	LTC6802_REG_FLG,
	LTC6802_REG_TMP
};

enum ltc6802_cfg_register {
	LTC6802_CFG_REG0,
	LTC6802_CFG_REG1,
	LTC6802_CFG_REG2,
	LTC6802_CFG_REG3,
	LTC6802_CFG_REG4,
	LTC6802_CFG_REG5,
};

enum ltc6802_status {
	LTC6802_STATUS_ADC,
	LTC6802_STATUS_INT
};

enum ltc6802_id {
	ltc6802
};

static const struct spi_device_id ltc6802_id[] = {
	{"ltc6802", ltc6802},
	{}
};
MODULE_DEVICE_TABLE(spi, ltc6802_id);

#ifdef CONFIG_OF
static const struct of_device_id ltc6802_adc_dt_ids[] = {
	{ .compatible = "ltc,ltc6802" },
	{},
};
MODULE_DEVICE_TABLE(of, ltc6802_adc_dt_ids);
#endif

#define LTC6802_V_CHAN(index)						\
	{								\
		.type = IIO_VOLTAGE,					\
		.indexed = 1,						\
		.channel = LTC6802_CHAN(index),				\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SCALE),	\
	}

#define LTC6802_DV_CHAN(index)						\
	{								\
		.type = IIO_VOLTAGE,					\
		.indexed = 1,						\
		.channel = LTC6802_CHAN(index),				\
		.channel2 = LTC6802_CHAN(index) - 1,			\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SCALE),	\
		.differential = 1,					\
	}

#define LTC6802_T_CHAN(index)						\
	{								\
		.type = IIO_TEMP,					\
		.indexed = 1,						\
		.channel = LTC6802_CHAN(index),				\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SCALE),	\
	}

static const struct iio_chan_spec ltc6802_channels[] = {
	LTC6802_T_CHAN(0),
	LTC6802_T_CHAN(1),
	LTC6802_T_CHAN(2),
	LTC6802_V_CHAN(0),
	LTC6802_DV_CHAN(1),
	LTC6802_DV_CHAN(2),
	LTC6802_DV_CHAN(3),
	LTC6802_DV_CHAN(4),
	LTC6802_DV_CHAN(5),
	LTC6802_DV_CHAN(6),
	LTC6802_DV_CHAN(7),
	LTC6802_DV_CHAN(8),
	LTC6802_DV_CHAN(9),
	LTC6802_DV_CHAN(10),
	LTC6802_DV_CHAN(11),
};

struct ltc6802_chip_info {
	const struct iio_chan_spec *channels;
	unsigned int num_channels;
};

static const struct ltc6802_chip_info ltc6802_chip_info_tbl[] = {
	[ltc6802] = {
		.channels = ltc6802_channels,
		.num_channels = ARRAY_SIZE(ltc6802_channels),
	},
};

struct ltc6802_state {
	const struct ltc6802_chip_info	*info;
	struct spi_device		*spi;
	struct mutex			lock;
	unsigned int			address;
	u8				cfg[6];
	/* Max Rx size is 8 bytes (when using WRCFG_CMD) */
	u8 				tx_buf[8]  ____cacheline_aligned;
	/* Max Tx size is 19 bytes (when using RDCV_CMD) */
	u8 				rx_buf[19] ____cacheline_aligned;
};

static u8 ltc6802_crc8(u8 crc_in, u8 data)
{
	u8 crc_out = crc_in ^ data;
	int i;

	for (i = 0; i < 8; i++) {
		if (crc_out & 0x80) {
			crc_out <<= 1;
			crc_out ^= 0x07;
		} else {
			crc_out <<= 1;
		}
	}

	return crc_out;
}

static u8 ltc6802_pec_calculation(u8 *buf, int size)
{
	u8 pec = 0;
	int i;

	for (i = 0; i < (size - 1); i++)
		pec = ltc6802_crc8(pec, buf[i]);

	return pec;
}

static int ltc6802_read_reg_group(struct iio_dev *indio_dev, int reg)
{
	int ret;
	int rx_size;
	u8 pec;
	struct ltc6802_state *st = iio_priv(indio_dev);
	struct spi_transfer xfers[] = {
		{
		       .tx_buf = st->tx_buf,
		       .len = 2,
		}, {
		       .rx_buf = st->rx_buf,
		},
	};

	st->tx_buf[0] = LTC6802_ADDR_CMD_SOF | st->address;
	switch(reg) {
	case LTC6802_REG_CFG:
		st->tx_buf[1] = LTC6802_CMD_RDCFG;
		rx_size = 7;
		break;
	case LTC6802_REG_CV:
		st->tx_buf[1] = LTC6802_CMD_RDCV;
		rx_size = 19;
		break;
	case LTC6802_REG_FLG:
		st->tx_buf[1] = LTC6802_CMD_RDFLG;
		rx_size = 4;
		break;
	case LTC6802_REG_TMP:
		st->tx_buf[1] = LTC6802_CMD_RDTMP;
		rx_size = 6;
		break;
	default:
		return -EINVAL;
	}

	xfers[1].len = rx_size;
	ret = spi_sync_transfer(st->spi, xfers, ARRAY_SIZE(xfers));
	if (ret) {
		dev_err(&indio_dev->dev,
			"Failed to read register group\n");
		return ret;
	}

	pec = ltc6802_pec_calculation(st->rx_buf, rx_size);
	if (pec != st->rx_buf[rx_size - 1]) {
		dev_err(&indio_dev->dev,
			"PEC error on register group\n");
		return -EINVAL;
	}

	/* Save cfg registers value */
	if (reg == LTC6802_REG_CFG) {
		st->cfg[0] = st->rx_buf[0];
		st->cfg[1] = st->rx_buf[1];
		st->cfg[2] = st->rx_buf[2];
		st->cfg[3] = st->rx_buf[3];
		st->cfg[4] = st->rx_buf[4];
		st->cfg[5] = st->rx_buf[5];
	}

	return 0;
}

static int ltc6802_get_discharge_value(int cell, u8 *buf)
{
	int bit;
	int reg;

	if (cell < 9) {
		bit = (cell - 1);
		reg = LTC6802_CFG_REG1;
	} else {
		bit = (cell - 9);
		reg = LTC6802_CFG_REG2;
	}

	return buf[reg] | (1 << bit);
}

static void ltc6802_set_discharge_value(bool set, int cell, u8 *buf)
{
	int bit;
	int reg;

	if (cell < 9) {
		bit = (cell - 1);
		reg = LTC6802_CFG_REG1;
	} else {
		bit = (cell - 9);
		reg = LTC6802_CFG_REG2;
	}

	if (set)
		buf[reg] |= (1 << bit);
	else
		buf[reg] &= ~(1 << bit);
}

static int ltc6802_get_gpio_value(int gpio, u8 *buf)
{
	return buf[LTC6802_CFG_REG0] & (1 << (gpio + 4));
}

static void ltc6802_set_gpio_value(bool set, int gpio, u8 *buf)
{
	if (set)
		buf[LTC6802_CFG_REG0] |= (1 << (gpio + 4));
	else
		buf[LTC6802_CFG_REG0] &= ~(1 << (gpio + 4));

}

static int ltc6802_get_chan_value(int channel, u8 *buf)
{
	int idx;
	int val;

	if (channel % 2) {
		idx = (channel - 1) + ((channel - 1) / 2);
		val = ((buf[idx + 1] & 0x0F) << 8) | buf[idx];
	} else {
		idx = channel + (channel / 2) - 1;
		val = (buf[idx] << 4) | ((buf[idx - 1] & 0xF0) >> 4);
	}

	return val;
}

static int ltc6802_write_cfg(struct iio_dev *indio_dev)
{
	int ret;
	struct ltc6802_state *st = iio_priv(indio_dev);

	st->tx_buf[0] = LTC6802_ADDR_CMD_SOF | st->address;
	st->tx_buf[1] = LTC6802_CMD_WRCFG;
	st->tx_buf[2] = st->cfg[0];
	st->tx_buf[3] = st->cfg[1];
	st->tx_buf[4] = st->cfg[2];
	st->tx_buf[5] = st->cfg[3];
	st->tx_buf[6] = st->cfg[4];
	st->tx_buf[7] = st->cfg[5];
	ret = spi_write(st->spi, &st->tx_buf, 8);
	if (ret)
		dev_err(&indio_dev->dev, "Failed to write configuration\n");

	return ret;
}

static int ltc6802_is_standby(struct iio_dev *indio_dev)
{
	int ret;
	struct ltc6802_state *st = iio_priv(indio_dev);

	ret = ltc6802_read_reg_group(indio_dev, LTC6802_REG_CFG);
	if (ret)
		return ret;

	return ((st->rx_buf[0] & LTC6802_CDC_MASK) == LTC6802_CFGR0_CDC_MODE0);
}

static int ltc6802_read_single_value(struct iio_dev *indio_dev,
				     struct iio_chan_spec const *chan, int *val)
{
	int ret;
	int reg;
	struct ltc6802_state *st = iio_priv(indio_dev);

	/*
	 * Device falls into standby mode if no activity is detected on the SCKI
	 * pin for 2.5 seconds. When in standby mode, the ADC is turned off so
	 * it needs to be waken up before requesting a conversion.
	 */
	if (!!ltc6802_is_standby(indio_dev)) {
		st->cfg[0] |= LTC6802_CFGR0_CDC_MODE1;
		ltc6802_write_cfg(indio_dev);
	}

	st->tx_buf[0] = LTC6802_ADDR_CMD_SOF | st->address;
	switch (chan->type) {
	case IIO_TEMP:
		st->tx_buf[1] = LTC6802_CMD_STTMPAD | chan->channel;
		reg = LTC6802_REG_TMP;
		break;
	case IIO_VOLTAGE:
		st->tx_buf[1] = LTC6802_CMD_STCVAD | chan->channel;
		reg = LTC6802_REG_CV;
		break;
	default:
		return -EINVAL;
	}

	ret = spi_write(st->spi, st->tx_buf, 2);
	if (ret) {
		dev_err(&indio_dev->dev,
			"Failed to request channel conversion\n");
		return ret;
	}

	/*
	 * Datasheet specifies a conversion time between 1 ms to 1.5 ms
	 * for a single channel. Tests have shown that the conversion of
	 * a given channel takes approx. 8.5 ms. The datasheet timing is
	 * respected only if the conversion of the same channel is requested
	 * again without having requested the conversion of another channel
	 * in the meantime.
	 */
	mdelay(10);
	
	ret = ltc6802_read_reg_group(indio_dev, reg);
	if (ret)
		return ret;

	*val = ltc6802_get_chan_value(chan->channel, st->rx_buf);

	return IIO_VAL_INT;
}

static int ltc6802_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long mask)
{
	int ret;
	struct ltc6802_state *st = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		mutex_lock(&st->lock);
		ret = ltc6802_read_single_value(indio_dev, chan, val);
		mutex_unlock(&st->lock);
		return ret;
	case IIO_CHAN_INFO_SCALE:
		if (chan->type != IIO_TEMP && chan->type != IIO_VOLTAGE)
			return -EINVAL;

		*val = LTC6802_INPUT_DELTA_MV;
		*val2 = LTC6802_ADC_RESOLUTION_BIT;
		return IIO_VAL_FRACTIONAL_LOG2;
	default:
		return -EINVAL;
	}
}

static const struct iio_info ltc6802_info = {
	.driver_module = THIS_MODULE,
	.read_raw = &ltc6802_read_raw,
};

static ssize_t ltc6802_pin_show(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
	int ret;
	int id;
	int val;
	const char *name = attr->attr.name;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct ltc6802_state *st = iio_priv(indio_dev);

	mutex_lock(&st->lock);
	ret = ltc6802_read_reg_group(indio_dev, LTC6802_REG_CFG);
	mutex_unlock(&st->lock);
	if (ret)
		return ret;

	id = LTC6802_ATTR_NAME_TO_ID(name);
	if (strstr(name, "gpio"))
		val = ltc6802_get_gpio_value(id, st->cfg);
	else
		val = ltc6802_get_discharge_value(id, st->cfg);

	return sprintf(buf, "%d\n", val);
}

static ssize_t ltc6802_pin_store(struct device *dev,
                                 struct device_attribute *attr, const char *buf,
                                 size_t count)
{
	int id;
	int val;
	const char *name = attr->attr.name;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct ltc6802_state *st = iio_priv(indio_dev);

	sscanf(buf, "%d\n", &val);

	mutex_lock(&st->lock);
	if (!!ltc6802_is_standby(indio_dev)) {
		st->cfg[0] |= LTC6802_CFGR0_CDC_MODE1;
		id = LTC6802_ATTR_NAME_TO_ID(attr->attr.name);
		if (strstr(name, "gpio"))
			ltc6802_set_gpio_value(val, id, st->cfg);
		else
			ltc6802_set_discharge_value(val, id, st->cfg);
		ltc6802_write_cfg(indio_dev);
	}
	mutex_lock(&st->lock);

	return count;
}

static LTC6802_DEVICE_ATTR(cell01_disch);
static LTC6802_DEVICE_ATTR(cell02_disch);
static LTC6802_DEVICE_ATTR(cell03_disch);
static LTC6802_DEVICE_ATTR(cell04_disch);
static LTC6802_DEVICE_ATTR(cell05_disch);
static LTC6802_DEVICE_ATTR(cell06_disch);
static LTC6802_DEVICE_ATTR(cell07_disch);
static LTC6802_DEVICE_ATTR(cell08_disch);
static LTC6802_DEVICE_ATTR(cell09_disch);
static LTC6802_DEVICE_ATTR(cell10_disch);
static LTC6802_DEVICE_ATTR(cell11_disch);
static LTC6802_DEVICE_ATTR(cell12_disch);

static LTC6802_DEVICE_ATTR(gpio01_pinctrl);
static LTC6802_DEVICE_ATTR(gpio02_pinctrl);

static struct attribute *dev_attrs[] = {
	&dev_attr_cell01_disch.attr,
	&dev_attr_cell02_disch.attr,
	&dev_attr_cell03_disch.attr,
	&dev_attr_cell04_disch.attr,
	&dev_attr_cell05_disch.attr,
	&dev_attr_cell06_disch.attr,
	&dev_attr_cell07_disch.attr,
	&dev_attr_cell08_disch.attr,
	&dev_attr_cell09_disch.attr,
	&dev_attr_cell10_disch.attr,
	&dev_attr_cell11_disch.attr,
	&dev_attr_cell12_disch.attr,
	&dev_attr_gpio01_pinctrl.attr,
	&dev_attr_gpio02_pinctrl.attr,
	NULL,
};
ATTRIBUTE_GROUPS(dev);

static int ltc6802_probe(struct spi_device *spi)
{
	int ret;
	const void *ltc6802_addr;
	struct iio_dev *indio_dev;
	struct ltc6802_state *st;

	dev_info(&spi->dev, "Probing..\n");

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev) {
		dev_err(&spi->dev, "Failed to allocate IIO device\n");
		return -ENOMEM;
	}

	spi_set_drvdata(spi, indio_dev);

	st = iio_priv(indio_dev);
	st->spi = spi;
	st->info = &ltc6802_chip_info_tbl[spi_get_device_id(spi)->driver_data];

	ltc6802_addr = of_get_property(st->spi->dev.of_node,
				       "ltc6802,address", NULL);
	if (!ltc6802_addr) {
		dev_err(&indio_dev->dev,
			"Failed to get serial interface address\n");
		return -EINVAL;
	}
	st->address = be32_to_cpup(ltc6802_addr);

	mutex_init(&st->lock);

	indio_dev->name = spi_get_device_id(spi)->name;
	indio_dev->dev.parent = &spi->dev;
	indio_dev->info = &ltc6802_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = st->info->channels;
	indio_dev->num_channels = st->info->num_channels;
	/* Index 0 is already used by chan_attr_group so use next one */
	indio_dev->groups[1] = dev_groups[0];

	ret = iio_device_register(indio_dev);
	if (ret) {
		dev_err(&indio_dev->dev, "Failed to register IIO device\n");
		return ret;
	}

	return 0;
}

static int ltc6802_remove(struct spi_device *spi)
{
	struct iio_dev *indio_dev = spi_get_drvdata(spi);

	dev_info(&spi->dev, "Removing..\n");

	iio_device_unregister(indio_dev);

	return 0;
}

static struct spi_driver ltc6802_driver = {
	.driver = {
		.name	= "ltc6802",
		.of_match_table = of_match_ptr(ltc6802_adc_dt_ids),
	},
	.probe		= ltc6802_probe,
	.remove		= ltc6802_remove,
	.id_table	= ltc6802_id,
};
module_spi_driver(ltc6802_driver);

MODULE_AUTHOR("Olivier C. Larocque <olivier.c.larocque@gmail.com>");
MODULE_DESCRIPTION("LTC6802 Battery Stack Monitor");
MODULE_LICENSE("GPL v2");
