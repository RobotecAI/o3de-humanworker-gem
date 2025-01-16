/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#include "HumanWorkerModuleInterface.h"

#include <AzCore/Memory/Memory.h>

#include <HumanWorker/AnimGraphInputProviderComponent.h>
#include <HumanWorker/NavigationMeshOrchestratorComponent.h>
#include <HumanWorker/NpcNavigatorComponent.h>
#include <HumanWorker/NpcPoseNavigatorComponent.h>
#include <HumanWorker/NpcWaypointNavigatorComponent.h>
#include <HumanWorker/WaypointComponent.h>
#include <HumanWorker/WaypointSelectorComponent.h>

namespace HumanWorker
{
    AZ_TYPE_INFO_WITH_NAME_IMPL(HumanWorkerModuleInterface, "HumanWorkerModuleInterface", "{683f2A43-01CD-4C45-9814-074AFFF69A70}");
    AZ_RTTI_NO_TYPE_INFO_IMPL(HumanWorkerModuleInterface, AZ::Module);
    AZ_CLASS_ALLOCATOR_IMPL(HumanWorkerModuleInterface, AZ::SystemAllocator);

    HumanWorkerModuleInterface::HumanWorkerModuleInterface()
    {
        m_descriptors.insert(
            m_descriptors.end(),
            {
                HumanWorker::AnimGraphInputProviderComponent::CreateDescriptor(),
                HumanWorker::NavigationMeshOrchestratorComponent::CreateDescriptor(),
                HumanWorker::WaypointComponent::CreateDescriptor(),
                HumanWorker::WaypointSelectorComponent::CreateDescriptor(),
                HumanWorker::NpcWaypointNavigatorComponent::CreateDescriptor(),
                HumanWorker::NpcPoseNavigatorComponent::CreateDescriptor(),
            });
    }
} // namespace HumanWorker
