/*
 * speaker-pa.c
 *
 * Copyright (C) 2012 SpreadTrum Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <sound/audio_pa.h>

paudio_pa_control audio_pa_amplifier;
EXPORT_SYMBOL_GPL(audio_pa_amplifier);
ssize_t pa_switch_show(struct class *class, struct class_attribute *attr, char *buf);
ssize_t pa_switch_store(struct class *class, struct class_attribute *attr, const char *buf, size_t count);

/* /sys/class/pa/xxx */
static struct class_attribute pa_class_attrs[] = {
	__ATTR(switch, 0664, pa_switch_show, pa_switch_store),
	__ATTR_NULL,
};

ssize_t pa_switch_show(struct class *class, struct class_attribute *attr, char *buf)
{
    return sprintf(buf, "%s<%d>|%s<%d>|%s<%d>\n",
			(audio_pa_amplifier && audio_pa_amplifier->earpiece.control) ? "0.earpiece" : "",
			(audio_pa_amplifier && audio_pa_amplifier->earpiece.control) ? audio_pa_amplifier->earpiece.control(-1, NULL) : -1,
			(audio_pa_amplifier && audio_pa_amplifier->headset.control) ? "1.headset" : "",
			(audio_pa_amplifier && audio_pa_amplifier->headset.control) ? audio_pa_amplifier->headset.control(-1, NULL) : -1,
			(audio_pa_amplifier && audio_pa_amplifier->speaker.control) ? "2.speaker" : "",
			(audio_pa_amplifier && audio_pa_amplifier->speaker.control) ? audio_pa_amplifier->speaker.control(-1, NULL) : -1);
}

ssize_t pa_switch_store(struct class *class, struct class_attribute *attr, const char *buf, size_t count)
{
	int type = buf[0];
	int cmd = buf[1];
	switch (type) {
	case 0:
		if (audio_pa_amplifier && audio_pa_amplifier->earpiece.control)
			audio_pa_amplifier->earpiece.control(cmd, NULL);
		else
			pr_warn("pa switch not available on <earpiece>\n");
		break;
	case 1:
		if (audio_pa_amplifier && audio_pa_amplifier->headset.control)
			audio_pa_amplifier->headset.control(cmd, NULL);
		else
			pr_warn("pa switch not available on <headset>\n");
		break;
	case 2:
		if (audio_pa_amplifier && audio_pa_amplifier->speaker.control)
			audio_pa_amplifier->speaker.control(cmd, NULL);
		else
			pr_warn("pa switch not available on <speaker>\n");
		break;
	default:
		pr_err("pa switch not support this type <0x%02x>\n", type);
		break;
	}
    return count;
}

struct class pa_class = {
	.name	= "pa",
	.owner	= THIS_MODULE,
	.class_attrs = pa_class_attrs,
};

static __devinit int speaker_pa_probe(struct platform_device *pdev)
{
	audio_pa_amplifier = platform_get_drvdata(pdev);
	if (audio_pa_amplifier) {
		if (audio_pa_amplifier->speaker.init)
			audio_pa_amplifier->speaker.init();
		if (audio_pa_amplifier->earpiece.init)
			audio_pa_amplifier->earpiece.init();
		if (audio_pa_amplifier->headset.init)
			audio_pa_amplifier->headset.init();
	}
	class_register(&pa_class);
	return 0;
}

static int __devexit speaker_pa_remove(struct platform_device *pdev)
{
	audio_pa_amplifier = NULL;
	class_unregister(&pa_class);
	return 0;
}

static struct platform_driver speaker_pa_driver = {
	.driver = {
		.name = "speaker-pa",
		.owner = THIS_MODULE,
	},
	.probe = speaker_pa_probe,
	.remove = __devexit_p(speaker_pa_remove),
};

static int __init speaker_pa_init(void)
{
	return platform_driver_register(&speaker_pa_driver);
}

arch_initcall(speaker_pa_init);

MODULE_DESCRIPTION("ALSA SoC Speaker PA Control");
MODULE_AUTHOR("Luther Ge <luther.ge@spreadtrum.com>");
MODULE_LICENSE("GPL");
