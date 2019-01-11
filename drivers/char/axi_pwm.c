#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <linux/device.h>

#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/slab.h>

#define AXI_PWM_MAJOR	99		/* Device major number		*/

#define DRIVER_NAME		"axi-pwm"

#define REG_CONTROL_OFFSET 0x00
#define REG_PRESCALE_OFFSET     0x04
#define REG_RESOLUTION_OFFSET   0x08
#define REG_DUTY_OFFSET         0x0C

#define pwm_readreg(offset)       readl_relaxed(id->membase + offset)
#define pwm_writereg(val, offset) writel_relaxed(val, id->membase + offset)

struct axi_pwm{
    struct list_head list;
    struct device		*dev;
    void __iomem *membase;
    int nr;
    
    unsigned int period;
    unsigned int resolution;
};
static struct class *axi_pwm_class;


static LIST_HEAD(axi_pwm_list);
static DEFINE_SPINLOCK(axi_pwm_list_lock);

static struct axi_pwm *axi_pwm_get_by_minor(unsigned index)
{
    struct axi_pwm *axi_pwm;

    spin_lock(&axi_pwm_list_lock);
    list_for_each_entry(axi_pwm, &axi_pwm_list, list) {
        if (axi_pwm->nr == index)
            goto found;
    }
    axi_pwm = NULL;
found:
    spin_unlock(&axi_pwm_list_lock);
    return axi_pwm;
}
static struct axi_pwm *get_free_axi_pwm(int nr)
{
    struct axi_pwm *axi_pwm;
    axi_pwm = kzalloc(sizeof(*axi_pwm), GFP_KERNEL);
    if (!axi_pwm)
        return ERR_PTR(-ENOMEM);
    axi_pwm->nr = nr;

    spin_lock(&axi_pwm_list_lock);
    list_add_tail(&axi_pwm->list, &axi_pwm_list);
    spin_unlock(&axi_pwm_list_lock);
    return axi_pwm;
}
#if 0
static void return_axi_pwm(struct axi_pwm *axi_pwm)
{
    spin_lock(&axi_pwm_list_lock);
    list_del(&axi_pwm->list);
    spin_unlock(&axi_pwm_list_lock);
    kfree(axi_pwm);
}

#endif

static const struct of_device_id axi_pwm_of_match[] = {
    { .compatible = "xlnx,AXI-PWM-1.0"},
    { /* end of table */ }
};
MODULE_DEVICE_TABLE(of, axi_pwm_of_match);

static int axi_pwm_open(struct inode *inode, struct file *file)
{
    unsigned int minor = iminor(inode);
    struct axi_pwm *id;
    printk("%s,%d\r\n",__FUNCTION__,minor);
    id = axi_pwm_get_by_minor(minor);
    if (!id)
        return -ENODEV;
    file->private_data = id;
    return 0;
}
static ssize_t axi_pwm_write(struct file *file, const char __user *buf,
                    size_t count, loff_t *offset)
{
    struct axi_pwm *id = file->private_data;
    char *tmp;
    printk("%s,%d\r\n",__FUNCTION__,id->nr);
    if (count > 8192)
        count = 8192;
    tmp = memdup_user(buf, count);
    pwm_writereg(tmp[0],REG_DUTY_OFFSET);
    kfree(tmp);
    return count;
}
static long axi_pwm_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    int data=arg;
    struct axi_pwm *id = file->private_data;
    printk("%s,%d\r\n",__FUNCTION__,id->nr);
    printk("cmd=%d,arg=%d\r\n",cmd,data);
    pwm_writereg(arg,cmd);
    return 0;
}

static int axi_pwm_release(struct inode *inode, struct file *file)
{
    struct axi_pwm *id = file->private_data;
    pwm_writereg(0,REG_DUTY_OFFSET);
    printk("%s,%d",__FUNCTION__,id->nr);
    return 0;
}

static const struct file_operations axi_pwm_fops = {
    .owner		= THIS_MODULE,
    /*.llseek		= no_llseek,
    .read		= axi_pwm_read,*/
    .write		= axi_pwm_write,
    .unlocked_ioctl	= axi_pwm_ioctl,
    .open		= axi_pwm_open,
    .release	= axi_pwm_release,
};

static int axi_pwm_probe(struct platform_device *pdev)
{
    struct resource *r_mem;
    struct axi_pwm *id;
    const struct of_device_id *match;
    int minor;
    int res;
    r_mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    minor = (r_mem->start>>16)&0x0f;
    id = get_free_axi_pwm(minor);
    id->nr = minor;
    printk("%s,nr:%d\r\n",__FUNCTION__,id->nr);

    id->dev = &pdev->dev;
    platform_set_drvdata(pdev, id);

    match = of_match_node(axi_pwm_of_match, pdev->dev.of_node);

    
    id->membase = devm_ioremap_resource(&pdev->dev, r_mem);
    printk("r_mem=%x,membase=%x  \r\n",r_mem->start,(u32)id->membase);
    if (IS_ERR(id->membase))
        return PTR_ERR(id->membase);
    
    printk("name:%s,full_name:%s\r\n",pdev->dev.of_node->name,pdev->dev.of_node->full_name);
    //id->nr = of_alias_get_id(pdev->dev.of_node, "AXI_PWM");
    //printk("%s,nr:%d\r\n",__FUNCTION__,id->nr);
    
    id->dev = device_create(axi_pwm_class, NULL, MKDEV(AXI_PWM_MAJOR, id->nr),
                                NULL, "axi_pwm-%d",id->nr);
    /*dev_set_name(id->dev,"axi_pwm-%d",id->nr);
    res = device_register(id->dev);
    if(res)
        return res;*/
    pwm_writereg(100*1000*1000,REG_PRESCALE_OFFSET);
    pwm_writereg(2,REG_RESOLUTION_OFFSET);
    pwm_writereg(1,REG_DUTY_OFFSET);
    
    printk("%s complete\r\n",__FUNCTION__);
    return 0;
}

static int axi_pwm_remove(struct platform_device *pdev)
{
    struct axi_pwm *id = platform_get_drvdata(pdev);
    devm_kfree(id->dev,id);
    return 0;
}

static struct platform_driver axi_pwm_drv = {
    .driver = {
        .name  = DRIVER_NAME,
        .of_match_table = axi_pwm_of_match,
        /*.pm = &axi_pwm_pm_ops,*/
    },
    .probe  = axi_pwm_probe,
    .remove = axi_pwm_remove,
};

static int __init axi_pwm_init(void)
{
    int res;
    printk("%s\r\n",__FUNCTION__);
    res = register_chrdev(AXI_PWM_MAJOR, "axi-pwm", &axi_pwm_fops);
    if (res)
        goto out;
    axi_pwm_class = class_create(THIS_MODULE, "axi_pwm");
    return platform_driver_register(&axi_pwm_drv);
out:
    printk(KERN_ERR "%s: Driver Initialisation failed\n", __FILE__);
    return res;
}
module_init(axi_pwm_init);

static void __exit axi_pwm_exit(void)
{
    class_destroy(axi_pwm_class);
    unregister_chrdev(AXI_PWM_MAJOR, "axi_pwm");
    platform_driver_unregister(&axi_pwm_drv);
}
module_exit(axi_pwm_exit);

MODULE_AUTHOR("ganggang");
MODULE_DESCRIPTION("axi-pwm driver");
MODULE_LICENSE("GPL");
