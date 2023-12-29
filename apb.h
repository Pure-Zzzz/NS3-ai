/*
 * Copyright (c) 2020-2023 Huazhong University of Science and Technology
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Authors: Pengyu Liu <eic_lpy@hust.edu.cn>
 *          Hao Yin <haoyin@uw.edu>
 *          Muyuan Shen <muyuan_shen@hust.edu.cn>
 */

#ifndef APB_H
#define APB_H

#include <cstdint>

struct EnvStruct
{
    uint32_t current_channel;
    uint32_t current_power;
    uint32_t current_disturbed_channel;
    uint32_t envtmp1=1;
    uint32_t envtmp2=2;
    uint32_t envtmp3=3;
    uint32_t envtmp4=4;
    float current_snr;
    uint32_t cpp_action=0;
};

struct ActStruct
{
    uint32_t next_channel;
    uint32_t next_power;
    uint32_t acttmp1=1;
    uint32_t acttmp2=2;
    uint32_t acttmp3=3;
    uint32_t acttmp4=4;
};

#endif // APB_H
