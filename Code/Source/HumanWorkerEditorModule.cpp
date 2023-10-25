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
    class HumanWorkerEditorModule : public HumanWorkerModuleInterface
    {
    public:
        AZ_RTTI(HumanWorkerEditorModule, "{5ef35fd6-72f8-45bc-8362-830b9eec87f8}", HumanWorkerModuleInterface);
        AZ_CLASS_ALLOCATOR(HumanWorkerEditorModule, AZ::SystemAllocator);

        HumanWorkerEditorModule()
        {
            m_descriptors.insert(m_descriptors.end(), {});
        }
    };
} // namespace HumanWorker

AZ_DECLARE_MODULE_CLASS(Gem_HumanWorker, HumanWorker::HumanWorkerEditorModule)
