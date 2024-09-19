#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/signal.h>
#include <linux/uaccess.h>
#include <linux/wait.h>

#define DRIVER_NAME "qemu_edu"
#define BAR 0
#define BASEMINOR 0
#define DEVICE_COUNTS 1

#define EDU_ID 0x00
#define EDU_LIVE_CHECK 0x04
#define EDU_FACT_CALC 0x08
#define EDU_STATUS 0x20
#define EDU_IRQ_STATUS 0x24
#define EDU_IRQ_RAISE 0x60
#define EDU_IRQ_ACK 0x64
#define EDU_DMA_SRC_ADDR 0x80
#define EDU_DMA_DST_ADDR 0x88
#define EDU_DMA_COUNT 0x90
#define EDU_DMA_CMD 0x98
#define EDU_BUFFER_ADDRESS 0x40000

// Custom Parameters
#define EDU_DMA_GET 0x1234

#define STATUS_COMPUTING 0x01
#define STATUS_IRQFACT 0x80

#define DMA_START 0x01
#define DMA_RAM2EDU 0x0
#define DMA_EDU2RAM 0x02
#define DMA_IRQ 0x04
#define DMA_IRQ_VALUE 0x100

#define BUFFER_SIZE 4096

#define SET_DMA(value64, pos)                                 \
    do {                                                      \
        iowrite32((uint32_t)((value64)&0xFFFFFFFF),           \
                  edu_dev->mmio_base + (pos));                \
        iowrite32((uint32_t)(((value64) >> 32) & 0xFFFFFFFF), \
                  edu_dev->mmio_base + (pos) + 4);            \
    } while (0)

static struct pci_device_id pci_ids[] = {{PCI_DEVICE(0x1234, 0x11e8)},
                                         {
                                             0,
                                         }};
MODULE_DEVICE_TABLE(pci, pci_ids);

struct edu_device {
    dev_t dev_num;
    struct pci_dev *pdev;
    void __iomem *mmio_base;

    struct cdev cdev;
    bool complete;
    wait_queue_head_t wait_queue;

    uint64_t dma_src_address;
    uint64_t dma_dst_address;
    uint64_t dma_count;
    void *dma_buffer;
    uint32_t dma_direction;
    dma_addr_t dma_addr;

    struct pid *user_pid;
    struct work_struct free_dma_work;
};

void free_dma_work_fn(struct work_struct *work) {
    struct edu_device *edu_dev =
        container_of(work, struct edu_device, free_dma_work);

    dma_free_coherent(&edu_dev->pdev->dev, edu_dev->dma_count,
                      edu_dev->dma_buffer, edu_dev->dma_addr);
}

static irqreturn_t edu_irq_handler(int irq, void *dev_id) {
    uint32_t status;
    struct edu_device *edu_dev = dev_id;

    // Check:
    // 1. dev_id is not null;
    // 2. Interrupt status register is not zero.
    if (!edu_dev ||
        (status = ioread32(edu_dev->mmio_base + EDU_IRQ_STATUS)) == 0) {
        return IRQ_NONE;
    }

    printk(KERN_INFO "[%s] Receive interrupt, status: 0x%x\n", DRIVER_NAME,
           status);

    // Wake up so that edu_read() can return correct value.
    edu_dev->complete = true;
    wake_up_interruptible(&edu_dev->wait_queue);

    // Check if DMA interrupt
    if (edu_dev->user_pid && status == DMA_IRQ_VALUE) {
        struct task_struct *task;
        struct kernel_siginfo info;

        memset(&info, 0, sizeof(struct kernel_siginfo));
        info.si_signo = SIGUSR1;
        info.si_code = SI_QUEUE;
        info.si_int =
            edu_dev->dma_direction & DMA_EDU2RAM ? DMA_EDU2RAM : DMA_RAM2EDU;

        task = get_pid_task(edu_dev->user_pid, PIDTYPE_PID);
        if (task) {
            if (send_sig_info(SIGUSR1, &info, task) < 0) {
                printk(KERN_ERR
                       "[%s] DMA IRQ: Failed to send signal to pid %d\n",
                       DRIVER_NAME, task_pid_nr(task));
            }
            put_task_struct(task);
        }
        schedule_work(&edu_dev->free_dma_work);
    }

    // Acknowledge Interrupt
    iowrite32(status, edu_dev->mmio_base + EDU_IRQ_ACK);

    return IRQ_HANDLED;
}

struct class *edu_class;

static int edu_open(struct inode *inode, struct file *filp) {
    struct edu_device *edu_dev;

    edu_dev = container_of(inode->i_cdev, struct edu_device, cdev);

    // Store current process id
    edu_dev->user_pid = get_pid(task_pid(current));

    // Store edu_device point in filp->private_data
    filp->private_data = edu_dev;

    return 0;
}

static ssize_t edu_read(struct file *filp, char __user *buf, size_t count,
                        loff_t *ppos) {
    struct edu_device *edu_dev;
    uint32_t value32;

    // Get edu_device
    edu_dev = filp->private_data;

    // Check read size
    if ((*ppos < 0x80 && count != 4) ||
        (*ppos >= 0x80 && count != 4 && count != 8)) {
        return -EINVAL;
    }

    switch (*ppos) {
        case EDU_FACT_CALC: {
            // Get factorial computation result
            int ret;
            ret = wait_event_interruptible(edu_dev->wait_queue,
                                           edu_dev->complete);
            // Deal with signal interruptions
            if (ret == -ERESTARTSYS) {
                return ret;
            }
            // Read data
            value32 = ioread32(edu_dev->mmio_base + EDU_FACT_CALC);
            if (copy_to_user(buf, &value32, sizeof(value32))) {
                return -EFAULT;
            }
            break;
        }
        case EDU_DMA_GET: {
            // Get data back, ignore 'buf' and 'count' parameters
            if (copy_to_user((void __user *)edu_dev->dma_dst_address,
                             edu_dev->dma_buffer, edu_dev->dma_count)) {
                printk(KERN_ERR "[%s] DMA GET: Failed to copy_to_user\n",
                       DRIVER_NAME);
                return -EFAULT;
            }
            printk(KERN_INFO "[%s] DMA GET: Content: %s\n", DRIVER_NAME,
                   (char *)edu_dev->dma_buffer);
            break;
        }
        default:
            return -EINVAL;
    }

    *ppos += count;

    return count;
}

static ssize_t edu_write(struct file *filp, const char __user *buf,
                         size_t count, loff_t *ppos) {
    struct edu_device *edu_dev;
    uint32_t value32 = 0;
    uint64_t value64 = 0;

    // Get edu_device
    edu_dev = filp->private_data;

    // Read data first
    if (count == 4) {
        if (copy_from_user(&value32, buf, sizeof(value32))) {
            return -EFAULT;
        }
    } else if (count == 8 && *ppos >= EDU_DMA_SRC_ADDR) {
        if (copy_from_user(&value64, buf, sizeof(value64))) {
            return -EFAULT;
        }
    } else {
        return -EINVAL;
    }

    switch (*ppos) {
        case EDU_FACT_CALC:
            // Factorial computation
            edu_dev->complete = false;
            iowrite32(value32, edu_dev->mmio_base + EDU_FACT_CALC);
            break;
        case EDU_DMA_SRC_ADDR:
            edu_dev->dma_src_address = count == 4 ? value32 : value64;
            break;
        case EDU_DMA_DST_ADDR: {
            edu_dev->dma_dst_address = count == 4 ? value32 : value64;
            break;
        }
        case EDU_DMA_COUNT:
            edu_dev->dma_count = count == 4 ? value32 : value64;
            break;
        case EDU_DMA_CMD: {
            void *buffer_addr;
            dma_addr_t dma_addr;
            struct device *dev = &edu_dev->pdev->dev;
            uint64_t cmd = count == 4 ? value32 : value64;

            // Set transfer count
            size_t size = edu_dev->dma_count;
            SET_DMA(size, EDU_DMA_COUNT);

            // Prepare DMA buffer
            buffer_addr = dma_alloc_coherent(dev, size, &dma_addr, GFP_KERNEL);
            if (!buffer_addr) {
                printk(KERN_ERR
                       "[%s] DMA_CMD: Failed to allocate memory for dma\n",
                       DRIVER_NAME);
                return -ENOMEM;
            }

            edu_dev->dma_buffer = buffer_addr;
            edu_dev->dma_direction = cmd & DMA_EDU2RAM;
            edu_dev->dma_addr = dma_addr;

            if (cmd & DMA_EDU2RAM) {
                // EDU to RAM
                // Check transfer count
                if (edu_dev->dma_src_address < EDU_BUFFER_ADDRESS ||
                    edu_dev->dma_src_address + size >=
                        EDU_BUFFER_ADDRESS + BUFFER_SIZE) {
                    printk(KERN_ERR "[%s] DMA_CMD: Memory out of bounds\n",
                           DRIVER_NAME);
                    dma_free_coherent(dev, size, buffer_addr, dma_addr);
                    return -EFAULT;
                }

                SET_DMA(edu_dev->dma_src_address, EDU_DMA_SRC_ADDR);
                SET_DMA(dma_addr, EDU_DMA_DST_ADDR);
                printk(KERN_INFO
                       "[%s] Start DMA: Direction: EDU to RAM, Source Address: "
                       "0x%llx, Destination Address: 0x%llx, count:%ld\n",
                       DRIVER_NAME, edu_dev->dma_src_address, dma_addr, size);
            } else {
                // RAM to EDU
                // Check transfer count
                if (edu_dev->dma_dst_address < EDU_BUFFER_ADDRESS ||
                    edu_dev->dma_dst_address + size >=
                        EDU_BUFFER_ADDRESS + BUFFER_SIZE) {
                    printk(KERN_ERR "[%s] DMA_CMD: Memory out of bounds\n",
                           DRIVER_NAME);
                    dma_free_coherent(dev, size, buffer_addr, dma_addr);
                    return -EFAULT;
                }

                // Copy user data to buffer
                if (copy_from_user(
                        buffer_addr,
                        (const void __user *)edu_dev->dma_src_address,
                        edu_dev->dma_count)) {
                    printk(KERN_ERR "[%s] DMA_CMD: Failed to copy_from_user\n",
                           DRIVER_NAME);
                    dma_free_coherent(dev, size, buffer_addr, dma_addr);
                    return -EFAULT;
                }

                SET_DMA(dma_addr, EDU_DMA_SRC_ADDR);
                SET_DMA(edu_dev->dma_dst_address, EDU_DMA_DST_ADDR);
                printk(KERN_INFO
                       "[%s] Start DMA: Direction: RAM to EDU, Source Address: "
                       "0x%llx, Destination Address: 0x%llx, count:%ld, "
                       "Content: %s\n",
                       DRIVER_NAME, dma_addr, edu_dev->dma_dst_address, size,
                       (char *)buffer_addr);
            }

            // Start DMA
            SET_DMA(cmd | DMA_START | DMA_IRQ, EDU_DMA_CMD);

            break;
        }
        default:
            return -EINVAL;
    }

    *ppos += count;
    return count;
}

static struct file_operations fops = {.owner = THIS_MODULE,
                                      .read = edu_read,
                                      .write = edu_write,
                                      .open = edu_open};

static int edu_probe(struct pci_dev *pdev, const struct pci_device_id *id) {
    struct edu_device *edu_dev;
    int ret = 0;
    dev_t dev_num;
    void __iomem *mmio_base;

    // Allocate an edu_device structure
    edu_dev = kzalloc(sizeof(*edu_dev), GFP_KERNEL);
    if (!edu_dev) return -ENOMEM;
    edu_dev->pdev = pdev;

    // Enable the PCI device
    if ((ret = pci_enable_device(pdev)) < 0) {
        printk(KERN_ERR "[%s] pci_enable_device failed. \n", DRIVER_NAME);
        goto free_edu_device;
    }

    // Allocate the device major number
    if ((ret = alloc_chrdev_region(&dev_num, BASEMINOR, DEVICE_COUNTS,
                                   DRIVER_NAME)) < 0) {
        printk(KERN_ERR "[%s] Failed to allocate char device region\n",
               DRIVER_NAME);
        goto disable_device;
    }
    edu_dev->dev_num = dev_num;

    // Initialize cdev
    cdev_init(&edu_dev->cdev, &fops);
    edu_dev->cdev.owner = THIS_MODULE;

    // Add the cdev to the system
    if (cdev_add(&edu_dev->cdev, edu_dev->dev_num, DEVICE_COUNTS)) {
        printk(KERN_ERR "[%s] Failed to add cdev\n", DRIVER_NAME);
        goto unregister_chrdev;
    }

    // Create Class
    edu_class = class_create(THIS_MODULE, DRIVER_NAME);
    if (IS_ERR(edu_class)) {
        printk(KERN_ERR "[%s] Failed to create class\n", DRIVER_NAME);
        ret = PTR_ERR(edu_class);
        goto delete_cdev;
    }

    // Create device node: /dev/edu
    if (device_create(edu_class, NULL, edu_dev->dev_num, NULL, "edu") == NULL) {
        printk(KERN_ERR "[%s] Failed to create device node\n", DRIVER_NAME);
        ret = -EINVAL;
        goto destroy_class;
    }

    // Request MMIO/IOP resources
    if ((ret = pci_request_region(pdev, BAR, DRIVER_NAME)) < 0) {
        printk(KERN_ERR "[%s] pci_request_region failed. \n", DRIVER_NAME);
        goto destroy_device;
    }

    // Set the DMA mask size
    // EDU device supports only 28 bits by default
    if ((ret = dma_set_mask_and_coherent(&(pdev->dev), DMA_BIT_MASK(28))) < 0) {
        dev_warn(&pdev->dev, "[%s] No suitable DMA available\n", DRIVER_NAME);
        goto release_region;
    }

    // Map the BAR register
    mmio_base = pci_iomap(pdev, BAR, pci_resource_len(pdev, BAR));
    if (!mmio_base) {
        printk(KERN_ERR "[%s] Cannot iomap BAR\n", DRIVER_NAME);
        ret = -ENOMEM;
        goto release_region;
    }
    edu_dev->mmio_base = mmio_base;

    // Allow device to initiate DMA operations
    pci_set_master(pdev);

    // Register IRQ handler
    if ((ret = request_irq(pdev->irq, edu_irq_handler, IRQF_SHARED, DRIVER_NAME,
                           edu_dev)) < 0) {
        printk(KERN_ERR "[%s] Failed to request IRQ %d\n", DRIVER_NAME,
               pdev->irq);
        goto unmap_bar;
    }

    // Bind edu_dev to pci_dev
    pci_set_drvdata(pdev, edu_dev);

    // Device initialize
    // Raise interrupt after finishing factorial computation
    iowrite32(STATUS_IRQFACT, edu_dev->mmio_base + EDU_STATUS);
    init_waitqueue_head(&edu_dev->wait_queue);
    edu_dev->complete = true;
    INIT_WORK(&edu_dev->free_dma_work, free_dma_work_fn);

    printk(KERN_INFO "[%s] probe sucessfully.\n", DRIVER_NAME);
    return 0;

// Error handling
unmap_bar:
    pci_iounmap(pdev, mmio_base);
release_region:
    pci_release_region(pdev, BAR);
destroy_device:
    device_destroy(edu_class, edu_dev->dev_num);
destroy_class:
    class_destroy(edu_class);
delete_cdev:
    cdev_del(&edu_dev->cdev);
unregister_chrdev:
    unregister_chrdev_region(edu_dev->dev_num, DEVICE_COUNTS);
disable_device:
    pci_disable_device(pdev);
free_edu_device:
    kfree(edu_dev);
    return ret;
}

static void edu_remove(struct pci_dev *pdev) {
    struct edu_device *edu_dev = pci_get_drvdata(pdev);

    // Free IRQ
    free_irq(pdev->irq, edu_dev);

    // Unmap BAR memory
    pci_iounmap(pdev, edu_dev->mmio_base);

    // Release PCI I/O resources
    pci_release_region(pdev, BAR);

    // Destroy device node /dev/edu
    device_destroy(edu_class, edu_dev->dev_num);

    // Destroy class
    class_destroy(edu_class);

    // Delete cdev
    cdev_del(&edu_dev->cdev);

    // Unregister char device region
    unregister_chrdev_region(edu_dev->dev_num, DEVICE_COUNTS);

    // Disable PCI device
    pci_disable_device(pdev);

    // Free allocated device structure
    kfree(edu_dev);

    printk(KERN_INFO "[%s] removed.\n", DRIVER_NAME);
}

static struct pci_driver pci_driver = {
    .name = DRIVER_NAME,
    .id_table = pci_ids,
    .probe = edu_probe,
    .remove = edu_remove,
};

static int __init edu_init(void) {
    int ret;
    if ((ret = pci_register_driver(&pci_driver)) < 0) {
        printk(KERN_ERR "[%s] Init failed. \n", DRIVER_NAME);
        return ret;
    }

    printk(KERN_INFO "[%s] Init sucessfully. \n", DRIVER_NAME);
    return ret;
}

static void __exit edu_exit(void) {
    pci_unregister_driver(&pci_driver);
    printk(KERN_INFO "[%s] exited. \n", DRIVER_NAME);
}

module_init(edu_init);
module_exit(edu_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("QEMU EDU Device Driver");