*************************************************************************/

#pragma once

#include <px4_platform_common/posix.h>
#include <px4_platform_common/module_params.h>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>
#include <px4_platform_common/posix.h>
using namespace time_literals;

extern "C" __EXPORT int ftdi_comm_modules_main(int argc, char *argv[]);


