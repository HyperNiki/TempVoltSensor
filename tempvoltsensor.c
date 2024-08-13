#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/sysfs.h>
#include <linux/init.h>
#include <linux/slab.h>

#define REG_TEMP_START   0x00
#define REG_VOLT_START   0x03
#define REG_STATUS       0x07
#define REG_POWER_CTRL   0x08

#define POWER_OFF_CMD    0xAA

struct tempvoltsensor_data {
    struct i2c_client *client;
    struct gpio_desc *irq_gpio;
    int irq;
};

MODULE_AUTHOR("Nikita Ivanov");
MODULE_DESCRIPTION("I2C bus driver");
MODULE_LICENSE("GPL");

static ssize_t show_temperature(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct tempvoltsensor_data *data = i2c_get_clientdata(to_i2c_client(dev));
    int i, ret;
    u8 temp[3];

    for (i = 0; i < 3; i++) {
        ret = i2c_smbus_read_byte_data(data->client, REG_TEMP_START + i);
        if (ret < 0)
            return ret;
        temp[i] = ret;
    }

    return sprintf(buf, "%d\n", (temp[0] << 16) | (temp[1] << 8) | temp[2]);
}

static ssize_t show_voltage(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct tempvoltsensor_data *data = i2c_get_clientdata(to_i2c_client(dev));
    int i, ret;
    u8 volt[4];

    for (i = 0; i < 4; i++) {
        ret = i2c_smbus_read_byte_data(data->client, REG_VOLT_START + i);
        if (ret < 0)
            return ret;
        volt[i] = ret;
    }

    return sprintf(buf, "%d\n", (volt[0] << 24) | (volt[1] << 16) | (volt[2] << 8) | volt[3]);
}

static ssize_t power_control_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct tempvoltsensor_data *data = i2c_get_clientdata(to_i2c_client(dev));
    int ret;

    ret = i2c_smbus_write_byte_data(data->client, REG_POWER_CTRL, POWER_OFF_CMD);
    if (ret < 0)
        return ret;

    return count;
}


static DEVICE_ATTR(temperature, S_IRUGO, show_temperature, NULL);
static DEVICE_ATTR(voltage, S_IRUGO, show_voltage, NULL);
static DEVICE_ATTR(power_control, S_IWUSR, NULL, power_control_store);

static irqreturn_t tempvoltsensor_irq_handler(int irq, void *dev_id)
{
    struct tempvoltsensor_data *data = dev_id;
    int status;

    status = i2c_smbus_read_byte_data(data->client, REG_STATUS);
    if (status < 0) {
        dev_err(&data->client->dev, "Failed to read status register\n");
        return IRQ_HANDLED;
    }

    if (status & 0x01) {
        dev_warn(&data->client->dev, "Overheat detected\n");
    }

    if (status & 0x02) {
        dev_warn(&data->client->dev, "Voltage drop detected\n");
    }

    return IRQ_HANDLED;
}

static int tempvoltsensor_probe(struct i2c_client *client)
{
    struct tempvoltsensor_data *data;
    int ret;

    data = devm_kzalloc(&client->dev, sizeof(struct tempvoltsensor_data), GFP_KERNEL);
    if (!data)
        return -ENOMEM;

    data->client = client;
    i2c_set_clientdata(client, data);

    ret = sysfs_create_file(&client->dev.kobj, &dev_attr_temperature.attr);
    if (ret)
        return ret;

    ret = sysfs_create_file(&client->dev.kobj, &dev_attr_voltage.attr);
    if (ret)
        goto remove_temp;

    ret = sysfs_create_file(&client->dev.kobj, &dev_attr_power_control.attr);
    if (ret)
        goto remove_volt;

    data->irq_gpio = gpiod_get_optional(&client->dev, "irq", GPIOD_IN);
    if (IS_ERR(data->irq_gpio)) {
        ret = PTR_ERR(data->irq_gpio);
        goto remove_power;
    }

    if (data->irq_gpio) {
        data->irq = gpiod_to_irq(data->irq_gpio);
        ret = devm_request_threaded_irq(&client->dev, data->irq, NULL,
                                        tempvoltsensor_irq_handler, IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
                                        "tempvoltsensor_irq", data);
        if (ret)
            goto remove_power;
    }

    return 0;

remove_power:
    sysfs_remove_file(&client->dev.kobj, &dev_attr_power_control.attr);
remove_volt:
    sysfs_remove_file(&client->dev.kobj, &dev_attr_voltage.attr);
remove_temp:
    sysfs_remove_file(&client->dev.kobj, &dev_attr_temperature.attr);
    return ret;
}

static void tempvoltsensor_remove(struct i2c_client *client)
{
    sysfs_remove_file(&client->dev.kobj, &dev_attr_power_control.attr);
    sysfs_remove_file(&client->dev.kobj, &dev_attr_voltage.attr);
    sysfs_remove_file(&client->dev.kobj, &dev_attr_temperature.attr);
    return;
}

static const struct i2c_device_id tempvoltsensor_id[] = {
    { "TempVoltSensor", 0 },
    { }
};

MODULE_DEVICE_TABLE(i2c, tempvoltsensor_id);

static struct i2c_driver tempvoltsensor_driver = {
    .driver = {
        .name = "TempVoltSensor",
    },
    .probe = tempvoltsensor_probe,
    .remove = tempvoltsensor_remove,
    .id_table = tempvoltsensor_id,
};

module_i2c_driver(tempvoltsensor_driver);