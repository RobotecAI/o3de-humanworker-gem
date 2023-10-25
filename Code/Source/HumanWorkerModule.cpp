/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#include <HumanWorkerModuleInterface.h>

namespace HumanWorker
{
    class HumanWorkerModule : public HumanWorkerModuleInterface
    {
    public:
        AZ_RTTI(HumanWorkerModule, "{23FC0060-021A-4243-ABBE-9676837C2674}", HumanWorkerModuleInterface);
        AZ_CLASS_ALLOCATOR(HumanWorkerModule, AZ::SystemAllocator);
    };
} // namespace HumanWorker

AZ_DECLARE_MODULE_CLASS(Gem_HumanWorker, HumanWorker::HumanWorkerModule)
