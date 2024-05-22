/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root
 * of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/RTTI/BehaviorContext.h>
#include <HumanWorker/NpcNavigatorBus.h>

namespace ROS2::HumanWorker
{
    void NpcNavigatorRequests::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::BehaviorContext* behaviorContext = azrtti_cast<AZ::BehaviorContext*>(context))
        {
            behaviorContext->EBus<NpcNavigatorRequestBus>("NpcNavigatorBus")
                ->Attribute(AZ::Script::Attributes::Category, "HumanWorker")
                ->Attribute(AZ::Script::Attributes::Scope, AZ::Script::Attributes::ScopeFlags::Common)
                ->Attribute(AZ::Script::Attributes::Module, "HumanWorker")
                ->Event("SetLinearSpeed", &NpcNavigatorRequestBus::Events::SetLinearSpeed, { { { "Linear speed", "" } } })
                ->Event("SetAngularSpeed", &NpcNavigatorRequestBus::Events::SetAngularSpeed, { { { "Angular speed", "" } } });
        }
    }
} // namespace ROS2::HumanWorker
