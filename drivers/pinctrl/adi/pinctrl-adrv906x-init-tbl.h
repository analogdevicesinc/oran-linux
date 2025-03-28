/*
 * Copyright (c) 2025, Analog Devices Incorporated, All Rights Reserved
 *
 * SPDX-License-Identifier: GPL-2.0+
 */

#ifndef __DRIVERS_PINCTRL_ADRV906X_INIT_TBL_H
#define __DRIVERS_PINCTRL_ADRV906X_INIT_TBL_H

#include <dt-bindings/pinctrl/pinctrl-adi-adrv906x-io-pad.h>
#include <linux/pinctrl/pinctrl.h>
#include "pinctrl-adi.h"

const struct pinctrl_pin_desc adi_adrv906x_pinctrl_pads[] = {
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_0),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_1),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_2),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_3),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_4),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_5),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_6),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_7),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_8),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_9),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_10),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_11),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_12),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_13),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_14),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_15),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_16),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_17),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_18),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_19),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_20),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_21),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_22),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_23),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_24),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_25),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_26),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_27),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_28),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_29),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_30),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_31),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_32),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_33),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_34),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_35),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_36),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_37),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_38),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_39),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_40),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_41),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_42),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_43),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_44),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_45),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_46),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_47),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_48),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_49),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_50),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_51),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_52),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_53),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_54),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_55),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_56),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_57),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_58),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_59),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_60),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_61),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_62),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_63),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_64),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_65),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_66),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_67),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_68),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_69),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_70),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_71),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_72),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_73),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_74),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_75),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_76),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_77),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_78),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_79),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_80),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_81),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_82),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_83),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_84),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_85),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_86),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_87),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_88),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_89),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_90),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_91),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_92),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_93),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_94),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_95),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_96),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_97),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_98),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_99),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_100),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_101),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_102),
	/* Dedicated IO */
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_115),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_116),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_117),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_118),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_119),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_120),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_121),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_122),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_123),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_124),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_125),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_126),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_127),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_128),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_129),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_130),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_131),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_132),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_133),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_134),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_135),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_136),
	ADI_PINCTRL_PIN(ADI_ADRV906X_PIN_137),
};

#endif /* __DRIVERS_PINCTRL_ADRV906X_INIT_TBL_H */
