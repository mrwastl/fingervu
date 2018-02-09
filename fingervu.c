/*
 * FingerVU touch and IR/keys driver
 *
 * Copyright (C) 2013 Wolfgang Astleitner (mrwastl@users.sourceforge.net)
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License as
 *	published by the Free Software Foundation, version 2.
 *
 */

#define DEBUG
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/usb.h>
#include <linux/usb/input.h>
#include <linux/hid.h>


#define DRIVER_VERSION "v0.1"
#define DRIVER_AUTHOR  "Wolfgang Astleitner (mrwastl@users.sourceforge.net)"
#define DRIVER_DESC    "SoundGraph FingerVU touch and IR/keys driver"

static bool debug;
module_param(debug, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(debug, "Debug messages: 0=no, 1=yes (default: no)");

static int fingervu_probe(struct usb_interface *interface, const struct usb_device_id *id);
static void fingervu_disconnect(struct usb_interface *interface);

/* table of devices that work with this driver */
static struct usb_device_id id_table [] = {
  /* FingerVU 706 */
  { USB_DEVICE(0x15c2, 0x3480) },
  { },
};


struct fingervu_context {
  struct usb_device* udev;
};


static int fingervu_probe(struct usb_interface *interface, const struct usb_device_id *id) {
  struct usb_device *udev = interface_to_usbdev(interface);
  struct fingervu_context *dev = NULL;
  int retval = -ENOMEM;

//  if (debug)
    printk("entering fingervu_probe");

  dev = kmalloc(sizeof(struct fingervu_context), GFP_KERNEL);
  if (dev == NULL) {
    dev_err(&interface->dev, "Unable to get memory for FingerVU struct\n");
    kfree(dev);
    return retval;
  }
  memset (dev, 0x00, sizeof (*dev));

  dev->udev = usb_get_dev(udev);

  usb_set_intfdata (interface, dev);

#if 0
  device_create_file(&interface->dev, &dev_attr_blue);
  device_create_file(&interface->dev, &dev_attr_red);
  device_create_file(&interface->dev, &dev_attr_green);
#endif

  dev_info(&interface->dev, "FingerVU device now attached\n");
  return 0;
}

static void fingervu_disconnect(struct usb_interface *interface) {
  struct fingervu_context *dev;

  dev = usb_get_intfdata (interface);
  usb_set_intfdata (interface, NULL);

#if 0
  device_remove_file(&interface->dev, &dev_attr_blue);
  device_remove_file(&interface->dev, &dev_attr_red);
  device_remove_file(&interface->dev, &dev_attr_green);
#endif

  usb_put_dev(dev->udev);

  kfree(dev);

  dev_info(&interface->dev, "FingerVU device now disconnected\n");
}

#if 0
static int __init fingervu_init(void) {
  int retval = 0;

  if (debug)
    printk("entering fingervu_init");

  retval = usb_register(&fingervu_driver);
  if (retval)
    pr_err("usb_register failed. Error number %d", retval);

  if (debug)
    printk("exiting fingervu_init");

  return retval;
}

static void __exit fingervu_exit(void) {
  if (debug)
    printk("entering fingervu_exit");

  usb_deregister(&fingervu_driver);
}
#endif

MODULE_DEVICE_TABLE (usb, id_table);

static struct usb_driver fingervu_driver = {
  .name =       "fingervu",
  .probe =      fingervu_probe,
  .disconnect = fingervu_disconnect,
  .id_table =   id_table,
};

//module_init (fingervu_init);
//module_exit (fingervu_exit);

module_usb_driver(fingervu_driver);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_VERSION(DRIVER_VERSION);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");

