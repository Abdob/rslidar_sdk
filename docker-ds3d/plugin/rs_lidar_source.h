// SPDX-License-Identifier: BSD-3-Clause
// Ds3D dataloader entry point for the RoboSense rs_driver (live UDP source).

#ifndef _DS3D_DATALOADER_RS_LIDAR_SOURCE_H
#define _DS3D_DATALOADER_RS_LIDAR_SOURCE_H

#include "ds3d/common/impl/impl_dataloader.h"

DS3D_EXTERN_C_BEGIN
DS3D_EXPORT_API ds3d::abiRefDataLoader* createRsLidarLoader();
DS3D_EXTERN_C_END

#endif
