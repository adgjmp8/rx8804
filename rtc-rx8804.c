#include <linux/bcd.h>
#include <linux/bitops.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/rtc.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>

/* Basic time and calendar register */
#define RX8804_SEC     		0x00
#define RX8804_MIN     		0x01
#define RX8804_HOUR    		0x02
#define RX8804_WEEK		0x03
#define RX8804_DAY		0x04
#define RX8804_MONTH   		0x05
#define RX8804_YEAR    		0x06
#define RX8804_RAM  		0x07
#define RX8804_MIN_ALARM	0x08
#define RX8804_HOUR_ALARM  	0x09
#define RX8804_WEEK_ALARM  	0x0A
#define RX8804_TIMER_COUNTER0 	0x0B
#define RX8804_TIMER_COUNTER1 	0x0C
#define RX8804_EXTENREG		0x0D
#define RX8804_FLAGREG    	0x0E
#define RX8804_CTRL		0x0F

#define RX8804_TIMESTAMP_SEC	0x10
#define RX8804_TIMESTAMP_MIN	0x11
#define RX8804_TIMESTAMP_HOUR	0x12
#define RX8804_TIMESTAMP_WEEK	0x13
#define RX8804_TIMESTAMP_DAYS	0x14
#define RX8804_TIMESTAMP_MONTHS	0x15
#define RX8804_TIMESTAMP_YEARS	0x16
#define RX8804_EVIN_CTRL    	0x17
#define RX8804_EVIN_MONITOR    	0x18
#define RX8804_SOUT_CTRL1   	0x19
#define RX8804_SOUT_CTRL2	0x1A
#define RX8804_TIMER_CONTORL   	0x1B
#define RX8804_MONITOR_TIMER0  	0x1C
#define RX8804_MONITOR_TIMER1  	0x1D
#define RX8804_MONITOR_TIMER2  	0x1E
#define RX8804_TIMER_COUNTER2  	0x1F

///* 0x20 to 0x2F are user registers */
//#define RX8804_RESV30  0x30
//#define RX8804_RESV31  0x32
//#define RX8804_IRQ     0x32

#define RX8804_EXT_TSEL0	BIT(0)
#define RX8804_EXT_TSET1	BIT(1)
#define RX8804_EXT_FSEL0	BIT(2)
#define RX8804_EXT_FSEL1	BIT(3)
#define RX8804_EXT_TE 	 	BIT(4)
#define RX8804_EXT_USEL  	BIT(5)
#define RX8804_EXT_WADA  	BIT(6)
#define RX8804_EXT_TEST  	BIT(7)

#define RX8804_FLAG_VDET BIT(0)
#define RX8804_FLAG_VLF  BIT(1)
#define RX8804_FLAG_AF   BIT(3)
#define RX8804_FLAG_TF   BIT(4)
#define RX8804_FLAG_UF   BIT(5)

#define RX8804_CTRL_RESET  	BIT(0)
#define RX8804_CTRL_AIE  	BIT(3)
#define RX8804_CTRL_TIE  	BIT(4)
#define RX8804_CTRL_UIE  	BIT(5)
#define RX8804_CTRL_CSEL0 	BIT(6)
#define RX8804_CTRL_CESL1	BIT(7)

#define RX8804_TIMER_CTRL_TSTP	BIT(7)
#define RX8804_TIMER_CTRL_TRES	BIT(6)

#define RX8804_EXTEN_

#define RX8804_ALARM_AE  BIT(7)

static const struct i2c_device_id rx8804_id[] = {
	{ "rx8804", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, rx8804_id);

static const struct of_device_id rx8804_of_match[] = {
	{ .compatible = "epson,rx8804" },
	{ }
};
MODULE_DEVICE_TABLE(of, rx8804_of_match);

struct rx8804_data {
	struct i2c_client *client;
	struct rtc_device *rtc;
	u8 ctrlreg;
};

static irqreturn_t rx8804_irq_1_handler(int irq, void *dev_id)
{
	struct i2c_client *client = dev_id;
	struct rx8804_data *rx8804 = i2c_get_clientdata(client);
	int flagreg;

	mutex_lock(&rx8804->rtc->ops_lock);

	flagreg = i2c_smbus_read_byte_data(client, RX8804_FLAGREG);

	if (flagreg <= 0) {
		mutex_unlock(&rx8804->rtc->ops_lock);
		return IRQ_NONE;
	}

	if (flagreg & RX8804_FLAG_VLF)
		dev_warn(&client->dev, "IRQ Frequency stop detected\n");

	if (flagreg & RX8804_FLAG_TF) {
		flagreg &= ~RX8804_FLAG_TF;
		rtc_update_irq(rx8804->rtc, 1, RTC_PF | RTC_IRQF);
	}

	if (flagreg & RX8804_FLAG_AF) {
		flagreg &= ~RX8804_FLAG_AF;
		rtc_update_irq(rx8804->rtc, 1, RTC_AF | RTC_IRQF);
	}

	if (flagreg & RX8804_FLAG_UF) {
		flagreg &= ~RX8804_FLAG_UF;
		rtc_update_irq(rx8804->rtc, 1, RTC_UF | RTC_IRQF);
	}

	i2c_smbus_write_byte_data(client, RX8804_FLAGREG, flagreg);

	mutex_unlock(&rx8804->rtc->ops_lock);
	return IRQ_HANDLED;
}

static int rx8804_get_time(struct device *dev, struct rtc_time *dt)
{
	struct rx8804_data *rx8804 = dev_get_drvdata(dev);
	u8 date[7];
	int err;

	err = i2c_smbus_read_i2c_block_data(rx8804->client, RX8804_SEC,7, date);
	if (err != 7)
		return err < 0 ? err : -EIO;

	dt->tm_sec = bcd2bin(date[RX8804_SEC - RX8804_SEC] & 0x7f);
	dt->tm_min = bcd2bin(date[RX8804_MIN - RX8804_SEC] & 0x7f);
	dt->tm_hour = bcd2bin(date[RX8804_HOUR - RX8804_SEC] & 0x3f);
	dt->tm_mday = bcd2bin(date[RX8804_DAY - RX8804_SEC] & 0x3f);
	dt->tm_mon = bcd2bin(date[RX8804_MONTH - RX8804_SEC] & 0x1f) - 1;
	dt->tm_year = bcd2bin(date[RX8804_YEAR - RX8804_SEC]) + 100;
	dt->tm_wday = ffs(date[RX8804_WEEK - RX8804_SEC] & 0x7f);

	return rtc_valid_tm(dt);
}

static int rx8804_set_time(struct device *dev, struct rtc_time *dt)
{
	struct rx8804_data *rx8804 = dev_get_drvdata(dev);
	u8 date[7];
	int ctrl;
	int ret;

	if ((dt->tm_year < 100) || (dt->tm_year > 199))
		return -EINVAL;

	/* set RESET bit to "1" to prevent imter update int time setting. */
	ctrl = i2c_smbus_read_byte_data(rx8804->client, RX8804_CTRL);		
	if (ctrl < 0)
		return ctrl;
	rx8804->ctrlreg = ctrl | RX8804_CTRL_RESET;		
	ret = i2c_smbus_write_byte_data(rx8804->client, RX8804_CTRL,
					rx8804->ctrlreg);
	if (ret < 0)
		return ret;

	date[RX8804_SEC - RX8804_SEC] = bin2bcd(dt->tm_sec);
	date[RX8804_MIN - RX8804_SEC] = bin2bcd(dt->tm_min);
	date[RX8804_HOUR - RX8804_SEC] = bin2bcd(dt->tm_hour);
	date[RX8804_DAY - RX8804_SEC] = bin2bcd(dt->tm_mday);
	date[RX8804_MONTH - RX8804_SEC] = bin2bcd(dt->tm_mon + 1);
	date[RX8804_YEAR - RX8804_SEC] = bin2bcd(dt->tm_year - 100);
	date[RX8804_WEEK - RX8804_SEC] = bin2bcd(1 << dt->tm_wday);

	ret = i2c_smbus_write_i2c_block_data(rx8804->client,
					     RX8804_SEC, 7, date);
	if (ret < 0)
		return ret;

	return 0;
}
static int rx8804_init_client(struct i2c_client *client)
{
	int err;
	/* Initialize reserved registers as specified in Inspection sheet */
	err = i2c_smbus_write_byte_data(client, RX8804_MIN_ALARM, 0x00);
	if (err < 0)
		return err;

	err = i2c_smbus_write_byte_data(client, RX8804_HOUR_ALARM, 0x00);
	if (err < 0)
		return err;
	err = i2c_smbus_write_byte_data(client, RX8804_WEEK_ALARM, 0x00);
	if (err < 0)
		return err;
	err = i2c_smbus_write_byte_data(client, RX8804_TIMER_COUNTER0, 0x00);
	if (err < 0)
		return err;

	err = i2c_smbus_write_byte_data(client, RX8804_TIMER_COUNTER1, 0x00);
	if (err < 0)
		return err;

	err = i2c_smbus_write_byte_data(client, RX8804_EXTENREG, 0x00);
	if (err < 0)
		return err;

	err = i2c_smbus_write_byte_data(client, RX8804_FLAGREG, 0x00);
	if (err < 0)
		return err;

	err = i2c_smbus_write_byte_data(client, RX8804_CTRL, 0x40);
	if (err < 0)
		return err;

	err = i2c_smbus_write_byte_data(client, RX8804_EVIN_CTRL, 0x00);
	if (err < 0)
		return err;

	err = i2c_smbus_write_byte_data(client, RX8804_EVIN_MONITOR, 0x00);
	if (err < 0)
		return err;

	err = i2c_smbus_write_byte_data(client, RX8804_SOUT_CTRL1, 0x00);
	if (err < 0)
		return err;

	err = i2c_smbus_write_byte_data(client, RX8804_SOUT_CTRL2, 0x00);
	if (err < 0)
		return err;

	err = i2c_smbus_write_byte_data(client, RX8804_TIMER_CONTORL, 0x00);
	if (err < 0)
		return err;

	err = i2c_smbus_write_byte_data(client, RX8804_TIMER_COUNTER2, 0x00);
	if (err < 0)
		return err;

	return err;
}

static int rx8804_read_alarm(struct device *dev, struct rtc_wkalrm *t)
{
	struct rx8804_data *rx8804 = dev_get_drvdata(dev);
	struct i2c_client *client = rx8804->client;
	u8 alarmvals[3];
	int flagreg;
	int err;

	err = i2c_smbus_read_i2c_block_data(client, RX8804_MIN_ALARM, 3, alarmvals);
	if (err != 3)
		return err < 0 ? err : -EIO;

	flagreg = i2c_smbus_read_byte_data(client, RX8804_FLAGREG);
	if (flagreg < 0)
		return flagreg;

	t->time.tm_sec = 0;
	t->time.tm_min = bcd2bin(alarmvals[0] & 0x7f);
	t->time.tm_hour = bcd2bin(alarmvals[1] & 0x3f);

	if (!(alarmvals[2] & RX8804_ALARM_AE))
		t->time.tm_mday = bcd2bin(alarmvals[2] & 0x7f);

	t->enabled = !!(rx8804->ctrlreg & RX8804_CTRL_AIE);
	t->pending = (flagreg & RX8804_FLAG_AF) && t->enabled;

	return err;
}

static int rx8804_set_alarm(struct device *dev, struct rtc_wkalrm *t)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct rx8804_data *rx8804 = dev_get_drvdata(dev);
	u8 alarmvals[3];
	int extreg, flagreg;
	int err;

	flagreg = i2c_smbus_read_byte_data(client, RX8804_FLAGREG);
	if (flagreg < 0) {
		return flagreg;
	}

	if (rx8804->ctrlreg & (RX8804_CTRL_AIE | RX8804_CTRL_UIE)) {
		rx8804->ctrlreg &= ~(RX8804_CTRL_AIE | RX8804_CTRL_UIE);
		err = i2c_smbus_write_byte_data(rx8804->client, RX8804_CTRL,
						rx8804->ctrlreg);
		if (err < 0) {
			return err;
		}
	}

	flagreg &= ~RX8804_FLAG_AF;
	err = i2c_smbus_write_byte_data(rx8804->client, RX8804_FLAGREG, flagreg);
	if (err < 0)
		return err;

	alarmvals[0] = bin2bcd(t->time.tm_min);
	alarmvals[1] = bin2bcd(t->time.tm_hour);
	alarmvals[2] = bin2bcd(t->time.tm_mday);

	err = i2c_smbus_write_i2c_block_data(rx8804->client, RX8804_MIN_ALARM,
					     2, alarmvals);
	if (err < 0)
		return err;

	extreg = i2c_smbus_read_byte_data(client, RX8804_EXTENREG);
	if (extreg < 0)
		return extreg;

	extreg |= RX8804_EXT_WADA;
	err = i2c_smbus_write_byte_data(rx8804->client, RX8804_EXTENREG, extreg);
	if (err < 0)
		return err;

	if (alarmvals[2] == 0)
		alarmvals[2] |= RX8804_ALARM_AE;

	err = i2c_smbus_write_byte_data(rx8804->client, RX8804_WEEK_ALARM,
					alarmvals[2]);
	if (err < 0)
		return err;

	if (t->enabled) {
		if (rx8804->rtc->uie_rtctimer.enabled)
			rx8804->ctrlreg |= RX8804_CTRL_UIE;
		if (rx8804->rtc->aie_timer.enabled)
			rx8804->ctrlreg |=
				(RX8804_CTRL_AIE | RX8804_CTRL_UIE);

		err = i2c_smbus_write_byte_data(rx8804->client, RX8804_CTRL,
						rx8804->ctrlreg);
		if (err < 0)
			return err;
	}

	return 0;
}

static int rx8804_alarm_irq_enable(struct device *dev,
				   unsigned int enabled)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct rx8804_data *rx8804 = dev_get_drvdata(dev);
	int flagreg;
	u8 ctrl;
	int err;

	ctrl = rx8804->ctrlreg;

	if (enabled) {
		if (rx8804->rtc->uie_rtctimer.enabled)
			ctrl |= RX8804_CTRL_UIE;
		if (rx8804->rtc->aie_timer.enabled)
			ctrl |= (RX8804_CTRL_AIE | RX8804_CTRL_UIE);
	} else {
		if (!rx8804->rtc->uie_rtctimer.enabled)
			ctrl &= ~RX8804_CTRL_UIE;
		if (!rx8804->rtc->aie_timer.enabled)
			ctrl &= ~RX8804_CTRL_AIE;
	}

	flagreg = i2c_smbus_read_byte_data(client, RX8804_FLAGREG);
	if (flagreg < 0)
		return flagreg;

	flagreg &= ~RX8804_FLAG_AF;
	err = i2c_smbus_write_byte_data(rx8804->client, RX8804_FLAGREG, flagreg);
	if (err < 0)
		return err;

	if (ctrl != rx8804->ctrlreg) {
		rx8804->ctrlreg = ctrl;
		err = i2c_smbus_write_byte_data(rx8804->client, RX8804_CTRL,
						rx8804->ctrlreg);
		if (err < 0)
			return err;
	}

	return 0;
}

static int rx8804_ioctl(struct device *dev, unsigned int cmd, unsigned long arg)
{
	struct rx8804_data *rx8804 = dev_get_drvdata(dev);
	struct rtc_wkalrm alarm;
	struct rtc_time *dt;
	int err = 0;

        err = mutex_lock_interruptible(&rx8804->rtc->ops_lock);
        if (err)
                return err;

	switch (cmd) {

        case RTC_AIE_ON:	//This ioctl does not need an argument, and it can be used to enable the RTC alarm interrupt.
                mutex_unlock(&rx8804->rtc->ops_lock);
                return rx8804_alarm_irq_enable(dev, 1);

        case RTC_AIE_OFF:	//This ioctl does not need an argument, and it can be used to disable the RTC update interrupt.
                mutex_unlock(&rx8804->rtc->ops_lock);
                return rx8804_alarm_irq_enable(dev, 0);

        case RTC_UIE_ON:	//This ioctl does not need an argument, and it can be used to enable the RTC update interrupt.
                mutex_unlock(&rx8804->rtc->ops_lock);
                return rtc_update_irq_enable(rx8804->rtc, 1);

        case RTC_UIE_OFF:	//This ioctl does not need an argument, and it can be used to disable the RTC update interrupt.
                mutex_unlock(&rx8804->rtc->ops_lock);
                return rtc_update_irq_enable(rx8804->rtc, 0);

        case RTC_PIE_ON:
                err = rtc_irq_set_state(rx8804->rtc, NULL, 1);
		return 0;

        case RTC_PIE_OFF:
                err = rtc_irq_set_state(rx8804->rtc, NULL, 0);
		return 0;

	case RTC_ALM_READ:
                mutex_unlock(&rx8804->rtc->ops_lock);

                err = rtc_read_alarm(rx8804->rtc, &alarm);
		if (err < 0)
			return err;
		if (copy_to_user((void __user *)arg, &alarm.time, sizeof(dt)))
			err = -EFAULT;
		return err;

	case RTC_ALM_SET:
		mutex_unlock(&rx8804->rtc->ops_lock);

		if (copy_from_user(&alarm.time, (void __user *)arg, sizeof(dt)))
			return -EFAULT;

		alarm.enabled = 0;
		alarm.pending = 0;
		alarm.time.tm_wday = -1;
		alarm.time.tm_yday = -1;
		alarm.time.tm_isdst = -1;

		{
			time64_t now, then;

			err = rtc_read_time(rx8804->rtc, dt);
			if (err < 0)
				return err;
			now = rtc_tm_to_time64(dt);

			alarm.time.tm_mday = dt->tm_mday;
			alarm.time.tm_mon = dt->tm_mon;
			alarm.time.tm_year = dt->tm_year;
			err  = rtc_valid_tm(&alarm.time);
			if (err < 0)
				return err;
			then = rtc_tm_to_time64(&alarm.time);

			/* alarm may need to wrap into tomorrow */
			if (then < now) {
				rtc_time64_to_tm(now + 24 * 60 * 60, dt);
				alarm.time.tm_mday = dt->tm_mday;
				alarm.time.tm_mon = dt->tm_mon;
				alarm.time.tm_year = dt->tm_year;
			}
		}

		return rtc_set_alarm(rx8804->rtc, &alarm);

	default:
		return -ENOIOCTLCMD;
	}
}

static struct rtc_class_ops rx8804_rtc_ops = {
	.read_time = rx8804_get_time,
	.set_time = rx8804_set_time,
	.ioctl = rx8804_ioctl,
	.alarm_irq_enable = rx8804_alarm_irq_enable,
};

static int rx8804_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct rx8804_data *rx8804;
	int err = 0;
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA
		| I2C_FUNC_SMBUS_I2C_BLOCK)) {
		dev_err(&adapter->dev, "doesn't support required functionality\n");
		return -EIO;
	}

	rx8804 = devm_kzalloc(&client->dev, sizeof(struct rx8804_data),
			      GFP_KERNEL);
	if (!rx8804)
		return -ENOMEM;

	rx8804->client = client;
	i2c_set_clientdata(client, rx8804);

	err = rx8804_init_client(client);
	if (err)
		return err;
	else
		dev_info(&client->dev, "Init Success\n");

	if (client->irq > 0) {
		dev_info(&client->dev, "IRQ %d supplied\n", client->irq);
		err = devm_request_threaded_irq(&client->dev, client->irq, NULL,
						rx8804_irq_1_handler,
						IRQF_TRIGGER_LOW | IRQF_ONESHOT,
						"rx8804", client);

		if (err) {
			dev_err(&client->dev, "unable to request IRQ\n");
			client->irq = 0;
		} else {
			rx8804_rtc_ops.read_alarm = rx8804_read_alarm;
			rx8804_rtc_ops.set_alarm = rx8804_set_alarm;
			rx8804_rtc_ops.alarm_irq_enable = rx8804_alarm_irq_enable;
		}
	}
	else
	dev_info(&client->dev, "IRQ supplied fail\n");
	rx8804->rtc = devm_rtc_device_register(&client->dev, client->name,
		&rx8804_rtc_ops, THIS_MODULE);

	if (IS_ERR(rx8804->rtc)) {
		dev_err(&client->dev, "unable to register the class device\n");
		return PTR_ERR(rx8804->rtc);
	}

	rx8804->rtc->max_user_freq = 1;

	return err;
}
static int rx8804_remove(struct i2c_client *client)
{
	dev_info(&client->dev, "rx8804 remove\n");
        return 0;
}

static struct i2c_driver rx8804_driver = {
	.driver = {
		.name = "rtc-rx8804",
		.of_match_table = of_match_ptr(rx8804_of_match),
	},
	.probe		= rx8804_probe,
	.id_table	= rx8804_id,
	.remove		= rx8804_remove,
};
module_i2c_driver(rx8804_driver);

MODULE_AUTHOR("Sam Kim <sam@varikorea.co.kr>");
MODULE_DESCRIPTION("Epson RX8804 RTC driver");
MODULE_LICENSE("GPL v2");

