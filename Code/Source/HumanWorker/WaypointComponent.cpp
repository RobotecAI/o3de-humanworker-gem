/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root
 * of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#include <HumanWorker/WaypointComponent.h>

namespace HumanWorker
{
    void WaypointComponent::Reflect(AZ::ReflectContext* context)
    {
        WaypointConfiguration::Reflect(context);
        if (auto* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<WaypointComponent, AZ::Component>()->Version(0)->Field(
                "Waypoint Configuration", &WaypointComponent::m_configuration);

            if (AZ::EditContext* editContext = serialize->GetEditContext())
            {
                // clang-format off
                editContext
                    ->Class<WaypointComponent>("Waypoint", "Waypoint")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::Category, "HumanWorker")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->DataElement(AZ::Edit::UIHandlers::Default, &WaypointComponent::m_configuration, "Waypoint Configuration", "Waypoint Configuration");
                // clang-format on
            }
        }
    }

    void WaypointComponent::Activate()
    {
        WaypointRequestBus::Handler::BusConnect(GetEntityId());
    }

    void WaypointComponent::Deactivate()
    {
        WaypointRequestBus::Handler::BusDisconnect();
    }

    WaypointConfiguration WaypointComponent::GetConfiguration()
    {
        return m_configuration;
    }
} // namespace HumanWorker
